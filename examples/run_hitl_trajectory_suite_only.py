#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd
from pymavlink import mavutil


REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES_DIR = REPO_ROOT / "examples"
FIGURE_SCRIPT = REPO_ROOT / "experimental_validation" / "trajectory_comparison_figures.py"
STAGE_SCRIPT = REPO_ROOT / "experimental_validation" / "stage_hitl_tracking_logs.py"
PULL_SCRIPT = EXAMPLES_DIR / "pull_sdcard_logs_over_mavftp.py"
RESTART_SCRIPT = EXAMPLES_DIR / "restart_hitl_px4_clean_gui.sh"
TRAJ_HELPER = EXAMPLES_DIR / "run_hitl_px4_builtin_trajectory_minimal.py"

TRAJECTORIES = (
    (100, "hairpin"),
    (101, "lemniscate"),
    (102, "circle"),
    (103, "time_optimal_30s"),
    (104, "minimum_snap_50s"),
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run only the 5 built-in validation trajectories in HIL and generate comparison figures.")
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--run-root", default=str(REPO_ROOT / "hitl_runs" / "trajectory_suite_only_20260403"))
    ap.add_argument("--max-attempts", type=int, default=4)
    ap.add_argument("--stock-root", default=str(REPO_ROOT / "examples" / "paper_assets" / "stage1_inputs" / "stock_sitl_proxy"))
    ap.add_argument("--identified-root", default=str(REPO_ROOT / "examples" / "paper_assets" / "stage1_inputs" / "digital_twin_sitl"))
    ap.add_argument("--compare-label-hil", default="HIL proxy")
    ap.add_argument("--compare-label-identified", default="SITL identified")
    ap.add_argument("--keep-jmavsim", action="store_true")
    return ap.parse_args()


def run_cmd(cmd: list[str], *, log_path: Path | None = None, check: bool = True) -> subprocess.CompletedProcess:
    if log_path is None:
        return subprocess.run(cmd, text=True, check=check, capture_output=True)

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as handle:
        proc = subprocess.run(cmd, text=True, stdout=handle, stderr=subprocess.STDOUT, check=False)
    if check and proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd)
    return proc


def latest_matching_csv(root: Path, traj_id: int) -> Path | None:
    tracking_root = root / "tracking_logs"
    if not tracking_root.exists():
        return None
    matches = sorted(tracking_root.glob(f"t{traj_id}r*.csv"), key=lambda p: (p.stat().st_mtime_ns, p.name))
    return matches[-1] if matches else None


def compute_rmse(csv_path: Path) -> tuple[float, float, float]:
    df = pd.read_csv(csv_path)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(float)
    err = ((pos - ref) ** 2).sum(axis=1) ** 0.5
    rmse = float((err**2).mean() ** 0.5)
    max_err = float(err.max()) if len(err) else 0.0
    final_err = float(err[-1]) if len(err) else 0.0
    return rmse, max_err, final_err


def missing_cases(root: Path) -> list[str]:
    missing: list[str] = []
    for traj_id, case_name in TRAJECTORIES:
        if latest_matching_csv(root, traj_id) is None:
            missing.append(case_name)
    return missing


def current_cube_port() -> str:
    by_id = Path("/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00")
    if by_id.exists():
        return str(by_id)

    acm = sorted(Path("/dev").glob("ttyACM*"))
    if not acm:
        raise RuntimeError("CubeOrange serial port is not present")
    return str(acm[-1])


def graceful_stop_pattern(pattern: str, *, label: str, timeout_s: float = 10.0) -> None:
    proc = subprocess.run(["pgrep", "-f", pattern], text=True, capture_output=True, check=False)
    pids = [int(line.strip()) for line in proc.stdout.splitlines() if line.strip()]
    if not pids:
        return

    print(f"[trajectory-suite] requesting graceful stop for {label}: {' '.join(str(pid) for pid in pids)}", flush=True)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            alive.append(pid)
        if not alive:
            return
        time.sleep(0.25)

    raise RuntimeError(f"{label} did not exit cleanly after SIGTERM: {' '.join(str(pid) for pid in alive)}")


def graceful_close_jmavsim_gui(timeout_s: float = 15.0) -> None:
    proc = subprocess.run(["pgrep", "-f", r"java.*jmavsim_run.jar|jmavsim_run.sh -q -s -d "], text=True, capture_output=True, check=False)
    pids = [int(line.strip()) for line in proc.stdout.splitlines() if line.strip()]
    if not pids:
        return

    window_ids: list[str] = []
    if shutil.which("wmctrl"):
        listing = subprocess.run(["wmctrl", "-lx"], text=True, capture_output=True, check=False)
        for line in listing.stdout.splitlines():
            if "jmavsim" in line.lower():
                window_ids.append(line.split()[0])
        if window_ids:
            print(f"[trajectory-suite] closing jMAVSim window via wmctrl: {' '.join(window_ids)}", flush=True)
            for wid in window_ids:
                subprocess.run(["wmctrl", "-ic", wid], check=False)

    if not window_ids and shutil.which("xdotool"):
        search = subprocess.run(["xdotool", "search", "--name", "^jMAVSim$"], text=True, capture_output=True, check=False)
        window_ids = [line.strip() for line in search.stdout.splitlines() if line.strip()]
        if window_ids:
            print(f"[trajectory-suite] closing jMAVSim window via xdotool: {' '.join(window_ids)}", flush=True)
            for wid in window_ids:
                subprocess.run(["xdotool", "windowclose", wid], check=False)

    if not window_ids:
        raise RuntimeError("jMAVSim is running but no GUI window was found; refusing abrupt shutdown")

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            alive.append(pid)
        if not alive:
            return
        time.sleep(0.25)

    raise RuntimeError(f"jMAVSim GUI close did not stop the process cleanly: {' '.join(str(pid) for pid in alive)}")


def request_px4_safe_shutdown(endpoint: str) -> None:
    kwargs = {"autoreconnect": False}
    if endpoint.startswith("/dev/"):
        kwargs["baud"] = 57600
    try:
        mav = mavutil.mavlink_connection(endpoint, **kwargs)
        hb = mav.wait_heartbeat(timeout=3)
        if not hb:
            print("[trajectory-suite] no live heartbeat before graceful stop; skipping PX4 safe shutdown", flush=True)
            return

        for name, value, param_type in (
            ("TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ):
            mav.mav.param_set_send(
                mav.target_system,
                mav.target_component,
                name.encode("ascii"),
                float(value),
                param_type,
            )
            time.sleep(0.1)

        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0, 0, 0, 0, 0,
        )
        time.sleep(0.5)

        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        print("[trajectory-suite] sent PX4 safe shutdown (mode reset + force disarm + reboot) before stopping jMAVSim", flush=True)
    except Exception as exc:
        print(f"[trajectory-suite] skipping PX4 safe shutdown: {exc}", flush=True)
    finally:
        try:
            mav.close()
        except Exception:
            pass


def graceful_stop_hitl_session(endpoint: str) -> None:
    request_px4_safe_shutdown(endpoint)
    graceful_stop_pattern(r"run_hitl_px4_|run_hitl_full_stack_", label="HIL helpers")
    graceful_close_jmavsim_gui()
    graceful_stop_pattern(r"mavlink_serial_hub.py", label="mavlink serial hub")
    time.sleep(2.0)


def pull_sdcard_logs(port: str, baud: int, destination_dir: Path, log_path: Path) -> None:
    cmd = [
        sys.executable,
        str(PULL_SCRIPT),
        "--port",
        port,
        "--baud",
        str(baud),
        "--destination-dir",
        str(destination_dir),
        "--download-timeout",
        "30",
        "--retries",
        "2",
    ]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as handle:
        try:
            proc = subprocess.run(
                cmd,
                text=True,
                stdout=handle,
                stderr=subprocess.STDOUT,
                check=False,
                timeout=120,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"MAVFTP pull timed out on {port}") from exc
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd)


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    console_dir = run_root / "console_logs"
    figure_dir = run_root / "figures"
    staged_hil_root = run_root / "hil_staged"
    raw_pull_root = run_root / "raw_pull"
    run_root.mkdir(parents=True, exist_ok=True)
    console_dir.mkdir(parents=True, exist_ok=True)
    raw_pull_root.mkdir(parents=True, exist_ok=True)

    manifest: list[dict[str, object]] = []
    captured_cases: set[str] = set()

    for traj_id, case_name in TRAJECTORIES:
        existing_csv = latest_matching_csv(raw_pull_root, traj_id)
        if existing_csv is not None:
            rmse, max_err, final_err = compute_rmse(existing_csv)
            manifest.append(
                {
                    "case": case_name,
                    "traj_id": traj_id,
                    "attempt": 0,
                    "success": False,
                    "tracking_csv": str(existing_csv),
                    "rmse_m": rmse,
                    "max_err_m": max_err,
                    "final_err_m": final_err,
                    "reused_existing_capture": True,
                }
            )
            captured_cases.add(case_name)
            print(
                f"[trajectory-suite] reusing existing capture for {case_name}: "
                f"rmse={rmse:.3f}m max={max_err:.3f}m final={final_err:.3f}m",
                flush=True,
            )
            continue

        case_best: dict[str, object] | None = None
        for attempt in range(1, args.max_attempts + 1):
            restart_log = console_dir / f"{case_name}_attempt{attempt:02d}_restart.log"
            helper_log = console_dir / f"{case_name}_attempt{attempt:02d}_helper.log"
            pull_log = console_dir / f"{case_name}_attempt{attempt:02d}_pull.log"
            entry: dict[str, object] = {
                "case": case_name,
                "traj_id": traj_id,
                "attempt": attempt,
                "restart_log": str(restart_log),
                "helper_log": str(helper_log),
                "pull_log": str(pull_log),
                "success": False,
            }
            manifest.append(entry)

            print(f"[trajectory-suite] restarting clean HIL for {case_name} (traj {traj_id}) attempt {attempt}", flush=True)
            try:
                run_cmd(
                    [str(RESTART_SCRIPT), str(Path(args.px4_root).expanduser().resolve()), str(args.baud), args.endpoint],
                    log_path=restart_log,
                    check=True,
                )
            except Exception as exc:
                entry["error"] = f"restart_failed: {exc}"
                print(f"[trajectory-suite] restart failed for {case_name} attempt {attempt}: {exc}", flush=True)
                continue

            helper_cmd = [
                sys.executable,
                str(TRAJ_HELPER),
                "--endpoint",
                args.endpoint,
                "--traj-id",
                str(traj_id),
            ]

            print(f"[trajectory-suite] running {case_name} trajectory", flush=True)
            proc = run_cmd(helper_cmd, log_path=helper_log, check=False)
            entry["return_code"] = proc.returncode
            log_text = helper_log.read_text(encoding="utf-8", errors="replace")
            entry["summary_line"] = next(
                (line.strip() for line in log_text.splitlines() if "Built-in trajectory summary:" in line),
                "",
            )
            entry["ulg_paths"] = [line.split("[logger] ", 1)[1].strip() for line in log_text.splitlines() if "[logger] " in line]
            try:
                graceful_stop_hitl_session(args.endpoint)
            except Exception as exc:
                entry["shutdown_error"] = str(exc)
            port = current_cube_port()
            try:
                pull_sdcard_logs(port, args.baud, raw_pull_root, pull_log)
            except Exception as exc:
                entry["pull_error"] = str(exc)
            latest_csv = latest_matching_csv(raw_pull_root, traj_id)
            if latest_csv is not None:
                rmse, max_err, final_err = compute_rmse(latest_csv)
                entry["tracking_csv"] = str(latest_csv)
                entry["rmse_m"] = rmse
                entry["max_err_m"] = max_err
                entry["final_err_m"] = final_err
                entry["tracking_captured"] = True
                captured_cases.add(case_name)
                print(
                    f"[trajectory-suite] {case_name} tracking captured on attempt {attempt}: "
                    f"rmse={rmse:.3f}m max={max_err:.3f}m final={final_err:.3f}m",
                    flush=True,
                )
                if case_best is None or rmse < float(case_best["rmse_m"]):
                    case_best = entry

            if proc.returncode == 0 and "trajectory_reader built-in trajectory stable" in log_text:
                entry["success"] = True
                print(f"[trajectory-suite] {case_name} passed on attempt {attempt}", flush=True)
                break

            if latest_csv is not None:
                entry["error"] = "trajectory_helper_failed_but_tracking_captured"
                print(
                    f"[trajectory-suite] {case_name} did not pass helper checks on attempt {attempt}, "
                    "but tracking was captured; stopping retries for this case.",
                    flush=True,
                )
                break

            entry["error"] = "trajectory_helper_failed"
            print(f"[trajectory-suite] {case_name} failed on attempt {attempt}", flush=True)

        if case_best is not None:
            case_best["best_attempt_for_case"] = True
        manifest_path = run_root / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    manifest_path = run_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    if not args.keep_jmavsim:
        print("[trajectory-suite] leaving jMAVSim running to avoid abrupt Cube faults after data capture", flush=True)

    missing = missing_cases(raw_pull_root)
    if missing:
        print(
            json.dumps(
                {
                    "manifest": str(manifest_path),
                    "captured_cases": sorted(captured_cases),
                    "missing_cases": missing,
                    "status": "incomplete",
                },
                indent=2,
            ),
            flush=True,
        )
        return 1

    run_cmd(
        [
            sys.executable,
            str(STAGE_SCRIPT),
            "--source-root",
            str(raw_pull_root),
            "--out-root",
            str(staged_hil_root),
        ],
        check=True,
    )

    fig_cmd = [
        sys.executable,
        str(FIGURE_SCRIPT),
        "--stock-root",
        str(Path(args.stock_root).expanduser().resolve()),
        "--compare-root",
        str(Path(args.identified_root).expanduser().resolve()),
        "--compare-label",
        args.compare_label_identified,
        "--compare-root-2",
        str(staged_hil_root),
        "--compare-label-2",
        args.compare_label_hil,
        "--out-dir",
        str(figure_dir),
    ]
    fig_proc = run_cmd(fig_cmd, check=True)
    (figure_dir / "comparison_summary.stdout.json").write_text(fig_proc.stdout, encoding="utf-8")

    print(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "captured_cases": sorted(captured_cases),
                "figures": str(figure_dir),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
