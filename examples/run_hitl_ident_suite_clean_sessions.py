#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.hitl_catalog import campaign_ident_profiles, identification_duration_s
from pull_sdcard_logs_over_mavftp import collect_remote_files, download_file_via_port
from run_hitl_trajectory_suite_only import current_cube_port, graceful_stop_hitl_session, run_cmd, wait_for_cube_port


RESTART_SCRIPT = SCRIPT_DIR / "restart_hitl_px4_clean_gui.sh"
IDENT_HELPER = SCRIPT_DIR / "run_hitl_px4_builtin_ident_minimal.py"
PULL_SCRIPT = SCRIPT_DIR / "pull_sdcard_logs_over_mavftp.py"
USB_STREAM_SCRIPT = SCRIPT_DIR / "set_hitl_usb_actuator_stream.sh"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run built-in identification maneuvers one-by-one with a fresh jMAVSim session for each case.")
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--run-root", default=str(REPO_ROOT / "hitl_runs" / "ident_suite_clean_20260403"))
    ap.add_argument("--profiles", nargs="*", default=campaign_ident_profiles("identification_only"))
    ap.add_argument("--max-attempts", type=int, default=2)
    ap.add_argument("--usb-stream-rate", type=int, default=200)
    ap.add_argument("--keep-jmavsim", action="store_true")
    return ap.parse_args()


def latest_ident_csv(root: Path, profile: str) -> Path | None:
    group_root = root / "identification_logs"
    if not group_root.exists():
        return None
    matches = list(group_root.glob(f"{profile}_r*.csv"))
    matches = sorted(matches, key=lambda p: (p.stat().st_mtime_ns, p.name))
    return matches[-1] if matches else None


def analyze_ident_csv(csv_path: Path) -> dict[str, float | int | str]:
    df = pd.read_csv(csv_path)
    for column in ["timestamp_us", "ref_x", "ref_y", "ref_z", "pos_x", "pos_y", "pos_z", "ref_yaw", "yaw"]:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")
    df = df.dropna(subset=["timestamp_us", "ref_x", "ref_y", "ref_z", "pos_x", "pos_y", "pos_z"])
    rows = int(len(df))
    if rows == 0:
        return {
            "csv": str(csv_path),
            "rows": 0,
            "duration_s": 0.0,
            "ref_motion_span_m": 0.0,
            "pos_motion_span_m": 0.0,
        }

    duration_s = float((df["timestamp_us"].iloc[-1] - df["timestamp_us"].iloc[0]) / 1e6)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(float)
    ref_span = float(np.linalg.norm(np.ptp(ref, axis=0)))
    pos_span = float(np.linalg.norm(np.ptp(pos, axis=0)))
    ref_yaw_span = 0.0
    yaw_span = 0.0
    if "ref_yaw" in df.columns and "yaw" in df.columns:
        ref_yaw = np.unwrap(df["ref_yaw"].dropna().to_numpy(float))
        yaw = np.unwrap(df["yaw"].dropna().to_numpy(float))
        if ref_yaw.size:
            ref_yaw_span = float(ref_yaw.max() - ref_yaw.min())
        if yaw.size:
            yaw_span = float(yaw.max() - yaw.min())
    return {
        "csv": str(csv_path),
        "rows": rows,
        "duration_s": duration_s,
        "ref_motion_span_m": ref_span,
        "pos_motion_span_m": pos_span,
        "ref_yaw_span_rad": ref_yaw_span,
        "yaw_span_rad": yaw_span,
    }


def ident_capture_healthy(profile: str, metrics: dict[str, float | int | str]) -> tuple[bool, str]:
    expected = identification_duration_s(profile)
    rows = int(metrics["rows"])
    duration_s = float(metrics["duration_s"])
    ref_span = float(metrics["ref_motion_span_m"])
    pos_span = float(metrics["pos_motion_span_m"])
    ref_yaw_span = float(metrics.get("ref_yaw_span_rad", 0.0))
    yaw_span = float(metrics.get("yaw_span_rad", 0.0))

    min_rows = max(100, int(expected * 8))
    min_duration_s = max(8.0, expected * 0.60)
    min_motion_span_m = 0.05
    min_yaw_span_rad = 0.30

    if rows < min_rows:
        return False, f"rows too small ({rows} < {min_rows})"
    if duration_s < min_duration_s:
        return False, f"duration too short ({duration_s:.2f}s < {min_duration_s:.2f}s)"
    if profile == "yaw_sweep":
        if ref_yaw_span < min_yaw_span_rad:
            return False, f"reference yaw span too small ({ref_yaw_span:.3f}rad < {min_yaw_span_rad:.3f}rad)"
        if yaw_span < min_yaw_span_rad:
            return False, f"yaw span too small ({yaw_span:.3f}rad < {min_yaw_span_rad:.3f}rad)"
        return True, "healthy"
    if max(ref_span, pos_span) < min_motion_span_m:
        return False, f"motion span too small (ref={ref_span:.3f}m pos={pos_span:.3f}m)"
    return True, "healthy"


def list_remote_ident_logs(port: str, baud: int, profile: str) -> dict[str, int | None]:
    grouped = collect_remote_files(
        port,
        baud,
        10.0,
        {"identification_logs": "/fs/microsd/identification_logs"},
        (".csv",),
    )
    files = {}
    for name, size in grouped.get("identification_logs", []):
        if name.startswith(f"{profile}_"):
            files[name] = size
    return files


def ident_file_sort_key(profile: str, name: str) -> tuple[int, int, str]:
    match = re.match(rf"^{re.escape(profile)}_r(\d+)_(?P<stamp>[0-9a-fA-F]+)\.csv$", name)
    if not match:
        return (0, 0, name)
    return (int(match.group(1)), int(match.group("stamp"), 16), name)


def pull_ident_logs(
    port: str,
    baud: int,
    destination_dir: Path,
    log_path: Path,
    profile: str,
    before_files: dict[str, int | None],
    after_files: dict[str, int | None],
) -> Path:
    created = [(name, after_files[name]) for name in after_files if name not in before_files]
    candidates = created if created else list(after_files.items())
    if not candidates:
        raise RuntimeError(f"No remote identification log found for profile {profile}")

    selected_name, selected_size = max(candidates, key=lambda item: ident_file_sort_key(profile, item[0]))
    local_path = destination_dir / "identification_logs" / selected_name
    downloaded = download_file_via_port(
        port=port,
        baud=baud,
        heartbeat_timeout=10.0,
        remote_path=f"/fs/microsd/identification_logs/{selected_name}",
        local_path=local_path,
        expected_size=selected_size,
        retries=2,
        timeout=30.0,
    )
    log_payload = {
        "profile": profile,
        "before_files": sorted(before_files),
        "after_files": sorted(after_files),
        "created_files": sorted(name for name, _ in created),
        "selected_file": selected_name,
        "selected_size": selected_size,
        "downloaded_to": str(downloaded),
    }
    log_path.write_text(json.dumps(log_payload, indent=2), encoding="utf-8")
    return downloaded


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    console_dir = run_root / "console_logs"
    raw_pull_root = run_root / "raw_pull"
    manifest_path = run_root / "manifest.json"
    run_root.mkdir(parents=True, exist_ok=True)
    console_dir.mkdir(parents=True, exist_ok=True)
    raw_pull_root.mkdir(parents=True, exist_ok=True)

    profiles = args.profiles or campaign_ident_profiles("identification_only")
    manifest: list[dict[str, object]] = []
    profile_failed: dict[str, bool] = {profile: True for profile in profiles}

    for profile in profiles:
        case_best: dict[str, object] | None = None
        for attempt in range(1, args.max_attempts + 1):
            restart_log = console_dir / f"{profile}_attempt{attempt:02d}_restart.log"
            stream_log = console_dir / f"{profile}_attempt{attempt:02d}_usb_stream.log"
            helper_log = console_dir / f"{profile}_attempt{attempt:02d}_helper.log"
            pull_log = console_dir / f"{profile}_attempt{attempt:02d}_pull.log"
            entry: dict[str, object] = {
                "profile": profile,
                "attempt": attempt,
                "restart_log": str(restart_log),
                "usb_stream_log": str(stream_log),
                "helper_log": str(helper_log),
                "pull_log": str(pull_log),
                "success": False,
            }
            manifest.append(entry)

            before_remote_files: dict[str, int | None] = {}
            try:
                before_remote_files = list_remote_ident_logs(current_cube_port(), args.baud, profile)
                entry["before_remote_count"] = len(before_remote_files)
            except Exception as exc:
                entry["before_remote_error"] = str(exc)

            print(f"[ident-suite] restarting clean HIL for {profile} attempt {attempt}", flush=True)
            run_cmd(
                [str(RESTART_SCRIPT), str(Path(args.px4_root).expanduser().resolve()), str(args.baud), args.endpoint],
                log_path=restart_log,
                check=True,
            )
            run_cmd(
                [str(USB_STREAM_SCRIPT), str(args.usb_stream_rate)],
                log_path=stream_log,
                check=True,
            )

            helper_cmd = [sys.executable, str(IDENT_HELPER), "--endpoint", args.endpoint, "--profile", profile]

            if profile == "yaw_sweep":
                helper_cmd.extend([
                    "--direct-xy-limit", "0.40",
                    "--direct-z-tolerance", "0.60",
                    "--direct-tilt-limit-deg", "20",
                    "--custom-xy-limit", "0.35",
                    "--custom-z-tolerance", "0.10",
                    "--custom-tilt-limit-deg", "20",
                ])
            print(f"[ident-suite] running built-in identification profile {profile}", flush=True)
            proc = run_cmd(helper_cmd, log_path=helper_log, check=False)
            entry["return_code"] = proc.returncode
            log_text = helper_log.read_text(encoding="utf-8", errors="replace")
            entry["summary_line"] = next(
                (line.strip() for line in log_text.splitlines() if "Built-in identification summary:" in line),
                "",
            )
            entry["ulg_paths"] = [line.split("[logger] ", 1)[1].strip() for line in log_text.splitlines() if "[logger] " in line]

            try:
                graceful_stop_hitl_session(args.endpoint)
            except Exception as exc:
                entry["shutdown_error"] = str(exc)

            port = wait_for_cube_port()
            after_remote_files: dict[str, int | None] = {}
            latest_csv: Path | None = None
            try:
                after_remote_files = list_remote_ident_logs(port, args.baud, profile)
                entry["after_remote_count"] = len(after_remote_files)
                latest_csv = pull_ident_logs(
                    port,
                    args.baud,
                    raw_pull_root,
                    pull_log,
                    profile,
                    before_remote_files,
                    after_remote_files,
                )
            except Exception as exc:
                entry["pull_error"] = str(exc)
                print(f"[ident-suite] {profile} pull failed on attempt {attempt}: {exc}", flush=True)
            if latest_csv is not None:
                metrics = analyze_ident_csv(latest_csv)
                entry["ident_csv"] = str(latest_csv)
                entry.update(metrics)
                healthy, reason = ident_capture_healthy(profile, metrics)
                entry["capture_healthy"] = healthy
                entry["capture_health_reason"] = reason
                print(
                    f"[ident-suite] {profile} capture: rows={metrics['rows']} duration={metrics['duration_s']:.2f}s "
                    f"ref_span={metrics['ref_motion_span_m']:.3f}m pos_span={metrics['pos_motion_span_m']:.3f}m "
                    f"healthy={healthy} ({reason})",
                    flush=True,
                )
                case_best = entry

            if proc.returncode == 0 and entry.get("capture_healthy") is True:
                entry["success"] = True
                profile_failed[profile] = False
                print(f"[ident-suite] {profile} passed on attempt {attempt}", flush=True)
                break

            entry["error"] = "ident_helper_failed_or_capture_unhealthy"
            print(f"[ident-suite] {profile} failed on attempt {attempt}", flush=True)

        if case_best is not None:
            case_best["best_attempt_for_profile"] = True
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    if not args.keep_jmavsim:
        print("[ident-suite] leaving jMAVSim stopped after the last clean session", flush=True)

    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    failed = [profile for profile in profiles if profile_failed.get(profile, True)]
    print(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "profiles": profiles,
                "failed_profiles": failed,
            },
            indent=2,
        ),
        flush=True,
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
