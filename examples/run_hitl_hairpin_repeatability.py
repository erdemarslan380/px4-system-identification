#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics
import shutil
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (SCRIPT_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from experimental_validation.build_hitl_repeatability_review_bundle import build_bundle as build_review_bundle
from experimental_validation.trajectory_catalog import validation_trajectory_id_map
from pull_sdcard_logs_over_mavftp import collect_remote_files, download_file_via_port
from run_hitl_trajectory_suite_only import current_cube_port, graceful_stop_hitl_session, run_cmd


TRAJECTORY_SCRIPT = SCRIPT_DIR / "run_hitl_clean_gui_trajectory.sh"
DEFAULT_TRAJ_ID = 100
TRACKING_FILE_RE = re.compile(r"^t(?P<traj_id>\d+)r(?P<run>\d+)_(?P<stamp>[0-9a-fA-F]+)\.csv$")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run clean HIL trajectory repeats and build the HITL repeatability review page.")
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--traj-id", type=int, default=DEFAULT_TRAJ_ID, choices=sorted(validation_trajectory_id_map()))
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--run-root", default="")
    ap.add_argument("--pull-timeout", type=float, default=90.0)
    ap.add_argument("--post-run-soak", type=float, default=5.0)
    ap.add_argument("--review-out", default="")
    ap.add_argument("--label", default="HITL repeatability")
    ap.add_argument("--headless", action="store_true")
    return ap.parse_args()


def resolve_case_name(traj_id: int) -> str:
    return validation_trajectory_id_map()[traj_id].name


def acceptance_env(*, headless: bool) -> dict[str, str]:
    env = os.environ.copy()
    env["PX4_SYSID_HEADLESS"] = "1" if headless else "0"
    env.setdefault("PX4_HIL_PIN_CPUS", "0")
    env.setdefault("PX4_HIL_USE_RT", "0")
    env.setdefault("PX4_HIL_READY_SOAK_S", "3")
    return env


def parse_summary_line(log_text: str) -> dict[str, object]:
    summary_re = re.compile(
        r"Module-takeoff built-in trajectory summary: "
        r"traj_id=(?P<traj_id>\d+) "
        r"max_xy_err=(?P<max_xy_err>[0-9.]+)m "
        r"max_z_err=(?P<max_z_err>[0-9.]+)m "
        r"max_tilt=(?P<max_tilt>[0-9.]+)deg "
        r"final_xy=(?P<final_xy>[0-9.]+)m "
        r"final_z_err=(?P<final_z_err>[0-9.]+)m "
        r"final_tilt=(?P<final_tilt>[0-9.]+)deg "
        r"failsafe=(?P<failsafe>True|False)"
    )
    match = summary_re.search(log_text)
    if not match:
        raise RuntimeError("trajectory summary line not found")
    values = match.groupdict()
    return {
        "traj_id": int(values["traj_id"]),
        "max_xy_err": float(values["max_xy_err"]),
        "max_z_err": float(values["max_z_err"]),
        "max_tilt": float(values["max_tilt"]),
        "final_xy": float(values["final_xy"]),
        "final_z_err": float(values["final_z_err"]),
        "final_tilt": float(values["final_tilt"]),
        "failsafe": values["failsafe"] == "True",
    }


def compute_rmse(csv_path: Path) -> tuple[float, float, float, int, float]:
    import pandas as pd

    df = pd.read_csv(csv_path)
    numeric = df[pd.to_numeric(df["timestamp_us"], errors="coerce").notna()].copy()
    ref = numeric[["ref_x", "ref_y", "ref_z"]].to_numpy(float)
    pos = numeric[["pos_x", "pos_y", "pos_z"]].to_numpy(float)
    err = ((pos - ref) ** 2).sum(axis=1) ** 0.5
    rmse = float((err ** 2).mean() ** 0.5) if len(err) else 0.0
    max_err = float(err.max()) if len(err) else 0.0
    final_err = float(err[-1]) if len(err) else 0.0
    duration_s = 0.0
    if len(numeric.index) >= 2:
        duration_s = float((float(numeric["timestamp_us"].iloc[-1]) - float(numeric["timestamp_us"].iloc[0])) / 1e6)
    return rmse, max_err, final_err, int(len(numeric.index)), duration_s


def list_remote_tracking_logs(port: str, baud: int, traj_id: int) -> dict[str, int | None]:
    grouped = collect_remote_files(
        port,
        baud,
        10.0,
        {"tracking_logs": "/fs/microsd/tracking_logs"},
        (".csv",),
    )
    files = {}
    for name, size in grouped.get("tracking_logs", []):
        if name.startswith(f"t{traj_id}r"):
            files[name] = size
    return files


def tracking_file_sort_key(name: str) -> tuple[int, int, str]:
    match = TRACKING_FILE_RE.match(name)
    if not match:
        return (0, 0, name)
    return (int(match.group("run")), int(match.group("stamp"), 16), name)


def pull_tracking_log(
    port: str,
    baud: int,
    destination_dir: Path,
    traj_id: int,
    before_files: dict[str, int | None],
    after_files: dict[str, int | None],
) -> Path:
    created = [(name, after_files[name]) for name in after_files if name not in before_files]
    candidates = created if created else list(after_files.items())
    if not candidates:
        raise RuntimeError(f"No remote tracking log found for trajectory {traj_id}")
    selected_name, selected_size = max(candidates, key=lambda item: tracking_file_sort_key(item[0]))
    local_path = destination_dir / "tracking_logs" / selected_name
    return download_file_via_port(
        port=port,
        baud=baud,
        heartbeat_timeout=10.0,
        remote_path=f"/fs/microsd/tracking_logs/{selected_name}",
        local_path=local_path,
        expected_size=selected_size,
        retries=2,
        timeout=30.0,
    )


def wait_for_tracking_csv(
    *,
    baud: int,
    traj_id: int,
    before_files: dict[str, int | None],
    destination_dir: Path,
    timeout_s: float,
) -> Path:
    deadline = time.time() + timeout_s
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            port = current_cube_port()
            after_files = list_remote_tracking_logs(port, baud, traj_id)
            if not after_files:
                raise RuntimeError("remote tracking dir empty")
            return pull_tracking_log(
                port=port,
                baud=baud,
                destination_dir=destination_dir,
                traj_id=traj_id,
                before_files=before_files,
                after_files=after_files,
            )
        except Exception as exc:
            last_error = exc
            time.sleep(1.0)
    raise RuntimeError(f"tracking CSV did not become downloadable within {timeout_s:.1f}s: {last_error}")


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, object]], key: str) -> tuple[float, float]:
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    if not values:
        return float("nan"), float("nan")
    if len(values) == 1:
        return values[0], 0.0
    return statistics.fmean(values), statistics.pstdev(values)


def main() -> int:
    args = parse_args()
    case_name = resolve_case_name(args.traj_id)
    run_root = (
        Path(args.run_root).expanduser().resolve()
        if args.run_root
        else (REPO_ROOT / "hitl_runs" / f"{case_name}_repeatability").resolve()
    )
    review_out = (
        Path(args.review_out).expanduser().resolve()
        if args.review_out
        else (REPO_ROOT / "docs" / "hitl_validation" / f"{case_name}_repeatability").resolve()
    )
    console_dir = run_root / "console_logs"
    raw_pull_root = run_root / "raw_pull"
    console_dir.mkdir(parents=True, exist_ok=True)
    raw_pull_root.mkdir(parents=True, exist_ok=True)
    env = acceptance_env(headless=args.headless)

    per_run_rows: list[dict[str, object]] = []

    for repeat in range(1, args.runs + 1):
        helper_log = console_dir / f"hairpin_repeat_{repeat:02d}.log"
        row: dict[str, object] = {
            "repeat": repeat,
            "traj_id": args.traj_id,
            "name": case_name,
            "helper_log": str(helper_log),
        }
        print(f"[{case_name}-repeat] run {repeat}/{args.runs}", flush=True)

        try:
            before_remote = list_remote_tracking_logs(current_cube_port(), args.baud, args.traj_id)
        except Exception:
            before_remote = {}

        proc = run_cmd(
            [
                str(TRAJECTORY_SCRIPT),
                str(Path(args.px4_root).expanduser().resolve()),
                str(args.traj_id),
                args.endpoint,
                str(args.baud),
            ],
            log_path=helper_log,
            check=False,
            env=env,
        )
        row["return_code"] = proc.returncode
        log_text = helper_log.read_text(encoding="utf-8", errors="replace")
        try:
            row.update(parse_summary_line(log_text))
        except Exception as exc:
            row["summary_error"] = str(exc)

        try:
            if args.post_run_soak > 0.0:
                time.sleep(args.post_run_soak)
            latest_csv = wait_for_tracking_csv(
                baud=args.baud,
                traj_id=args.traj_id,
                before_files=before_remote,
                destination_dir=raw_pull_root / f"repeat_{repeat:02d}",
                timeout_s=args.pull_timeout,
            )
            rmse, max_err, final_err, csv_rows, duration_s = compute_rmse(latest_csv)
            row["tracking_csv"] = str(latest_csv)
            row["rmse_m"] = rmse
            row["max_err_m"] = max_err
            row["final_err_m"] = final_err
            row["csv_rows"] = csv_rows
            row["csv_duration_s"] = duration_s
            row["success"] = proc.returncode == 0
        except Exception as exc:
            row["pull_error"] = str(exc)
            row["success"] = False
            print(f"[{case_name}-repeat] repeat {repeat} pull failed: {exc}", flush=True)

        try:
            graceful_stop_hitl_session(args.endpoint)
        except Exception as exc:
            row["shutdown_error"] = str(exc)

        per_run_rows.append(row)
        write_csv(
            run_root / f"{case_name}_runs.csv",
            per_run_rows,
            [
                "repeat",
                "traj_id",
                "name",
                "return_code",
                "success",
                "helper_log",
                "tracking_csv",
                "rmse_m",
                "max_err_m",
                "final_err_m",
                "csv_rows",
                "csv_duration_s",
                "max_xy_err",
                "max_z_err",
                "max_tilt",
                "final_xy",
                "final_z_err",
                "final_tilt",
                "failsafe",
                "summary_error",
                "shutdown_error",
                "pull_error",
            ],
        )

    successful_csvs = [Path(str(row["tracking_csv"])) for row in per_run_rows if row.get("tracking_csv")]
    if len(successful_csvs) != args.runs:
        raise SystemExit(
            json.dumps(
                {
                    "ok": False,
                    "reason": "not_all_runs_captured",
                    "captured_runs": len(successful_csvs),
                    "requested_runs": args.runs,
                    "run_root": str(run_root),
                },
                indent=2,
            )
        )

    combined_tracking_dir = run_root / "combined_tracking_logs"
    combined_tracking_dir.mkdir(parents=True, exist_ok=True)
    for csv_path in successful_csvs:
        shutil.copy2(csv_path, combined_tracking_dir / csv_path.name)

    review_bundle = build_review_bundle(
        tracking_dir=combined_tracking_dir,
        out_dir=review_out,
        case_name=case_name,
        label=args.label,
        max_runs=args.runs,
        samples=700,
        max_points=1800,
    )

    summary = {
        "ok": True,
        "run_root": str(run_root),
        "review_out": review_bundle["out_dir"],
        "runs": len(per_run_rows),
        "rmse_mean_m": summarize(per_run_rows, "rmse_m")[0],
        "rmse_std_m": summarize(per_run_rows, "rmse_m")[1],
        "max_err_mean_m": summarize(per_run_rows, "max_err_m")[0],
        "max_err_std_m": summarize(per_run_rows, "max_err_m")[1],
    }
    (run_root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
