#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (SCRIPT_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from experimental_validation.hitl_catalog import campaign_ident_profiles
from pull_sdcard_logs_over_mavftp import collect_remote_files, download_file_via_port
from run_hitl_trajectory_suite_only import current_cube_port, graceful_stop_hitl_session, run_cmd, wait_for_cube_port


TRAJECTORY_SCRIPT = SCRIPT_DIR / "run_hitl_clean_gui_trajectory.sh"
IDENT_SUITE_SCRIPT = SCRIPT_DIR / "run_hitl_ident_suite_clean_sessions.py"
REVIEW_BUNDLE_SCRIPT = REPO_ROOT / "experimental_validation" / "build_hitl_review_bundle.py"
SUMMARY_FIGURES_SCRIPT = REPO_ROOT / "experimental_validation" / "build_hitl_acceptance_figures.py"

TRAJECTORIES: tuple[tuple[int, str], ...] = (
    (100, "hairpin"),
    (101, "lemniscate"),
    (102, "circle"),
    (103, "time_optimal_30s"),
    (104, "minimum_snap_50s"),
)

TRAJ_SUMMARY_RE = re.compile(
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

IDENT_SUMMARY_RE = re.compile(
    r"Built-in identification summary: "
    r"profile=(?P<profile>[a-z_]+) "
    r"max_xy=(?P<max_xy>[0-9.]+)m "
    r"max_z_err=(?P<max_z_err>[0-9.]+)m "
    r"max_tilt=(?P<max_tilt>[0-9.]+)deg "
    r"final_xy=(?P<final_xy>[0-9.]+)m "
    r"final_z_err=(?P<final_z_err>[0-9.]+)m "
    r"final_tilt=(?P<final_tilt>[0-9.]+)deg "
    r"failsafe=(?P<failsafe>True|False)"
)

TRACKING_FILE_RE = re.compile(r"^t(?P<traj_id>\d+)r(?P<run>\d+)_(?P<stamp>[0-9a-fA-F]+)\.csv$")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run repeated HIL acceptance campaigns for trajectories and identification maneuvers.")
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--run-root", default=str(REPO_ROOT / "hitl_runs" / "acceptance_repeats"))
    ap.add_argument("--trajectory-repeats", type=int, default=5)
    ap.add_argument("--ident-repeats", type=int, default=5)
    ap.add_argument("--skip-trajectories", action="store_true")
    ap.add_argument("--skip-ident", action="store_true")
    return ap.parse_args()


def acceptance_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("PX4_SYSID_HEADLESS", "1")
    env.setdefault("PX4_HIL_PIN_CPUS", "0")
    env.setdefault("PX4_HIL_USE_RT", "0")
    env.setdefault("PX4_HIL_READY_SOAK_S", "3")
    return env


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def parse_trajectory_summary(log_text: str) -> dict[str, object]:
    match = TRAJ_SUMMARY_RE.search(log_text)
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


def parse_ident_summary(summary_line: str) -> dict[str, object]:
    match = IDENT_SUMMARY_RE.search(summary_line)
    if not match:
        raise RuntimeError("identification summary line not found")
    values = match.groupdict()
    return {
        "profile": values["profile"],
        "max_xy": float(values["max_xy"]),
        "max_z_err": float(values["max_z_err"]),
        "max_tilt": float(values["max_tilt"]),
        "final_xy": float(values["final_xy"]),
        "final_z_err": float(values["final_z_err"]),
        "final_tilt": float(values["final_tilt"]),
        "failsafe": values["failsafe"] == "True",
    }


def mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    if len(values) == 1:
        return values[0], 0.0
    return statistics.fmean(values), statistics.pstdev(values)


def latest_matching_csv(root: Path, traj_id: int) -> Path | None:
    tracking_root = root / "tracking_logs"
    if not tracking_root.exists():
        return None
    matches = sorted(tracking_root.glob(f"t{traj_id}r*.csv"), key=lambda p: (p.stat().st_mtime_ns, p.name))
    return matches[-1] if matches else None


def compute_rmse(csv_path: Path) -> tuple[float, float, float, int, float]:
    import pandas as pd

    df = pd.read_csv(csv_path)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(float)
    err = ((pos - ref) ** 2).sum(axis=1) ** 0.5
    rmse = float((err**2).mean() ** 0.5) if len(err) else 0.0
    max_err = float(err.max()) if len(err) else 0.0
    final_err = float(err[-1]) if len(err) else 0.0
    duration_s = 0.0
    if len(df.index) >= 2 and "timestamp_us" in df.columns:
        duration_s = float((float(df["timestamp_us"].iloc[-1]) - float(df["timestamp_us"].iloc[0])) / 1e6)
    return rmse, max_err, final_err, int(len(df.index)), duration_s


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


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    ensure_parent(path)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_markdown_table(path: Path, headers: list[str], rows: list[list[object]]) -> None:
    ensure_parent(path)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("| " + " | ".join(headers) + " |\n")
        handle.write("| " + " | ".join(["---"] * len(headers)) + " |\n")
        for row in rows:
            handle.write("| " + " | ".join(str(item) for item in row) + " |\n")


def summarize_trajectory_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[int, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["traj_id"])].append(row)

    summaries: list[dict[str, object]] = []
    for traj_id, name in TRAJECTORIES:
        group = grouped.get(traj_id, [])
        success_rows = [row for row in group if bool(row["success"])]
        metrics = {
            "max_xy_err": [float(row["max_xy_err"]) for row in success_rows],
            "max_z_err": [float(row["max_z_err"]) for row in success_rows],
            "max_tilt": [float(row["max_tilt"]) for row in success_rows],
            "final_xy": [float(row["final_xy"]) for row in success_rows],
            "final_z_err": [float(row["final_z_err"]) for row in success_rows],
            "final_tilt": [float(row["final_tilt"]) for row in success_rows],
            "rmse_m": [float(row["rmse_m"]) for row in success_rows if row.get("rmse_m") is not None],
            "max_err_m": [float(row["max_err_m"]) for row in success_rows if row.get("max_err_m") is not None],
            "csv_duration_s": [float(row["csv_duration_s"]) for row in success_rows if row.get("csv_duration_s") is not None],
        }
        summary: dict[str, object] = {
            "traj_id": traj_id,
            "name": name,
            "runs": len(group),
            "successes": len(success_rows),
            "failures": len(group) - len(success_rows),
        }
        for key, values in metrics.items():
            mean_value, std_value = mean_std(values)
            summary[f"{key}_mean"] = mean_value
            summary[f"{key}_std"] = std_value
        summaries.append(summary)
    return summaries


def summarize_ident_rows(rows: list[dict[str, object]], profiles: list[str]) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["profile"])].append(row)

    summaries: list[dict[str, object]] = []
    for profile in profiles:
        group = grouped.get(profile, [])
        success_rows = [row for row in group if bool(row["success"])]
        metrics = {
            "max_xy": [float(row["max_xy"]) for row in success_rows],
            "max_z_err": [float(row["max_z_err"]) for row in success_rows],
            "max_tilt": [float(row["max_tilt"]) for row in success_rows],
            "rows": [float(row["rows"]) for row in success_rows],
            "duration_s": [float(row["duration_s"]) for row in success_rows],
            "ref_motion_span_m": [float(row["ref_motion_span_m"]) for row in success_rows],
            "pos_motion_span_m": [float(row["pos_motion_span_m"]) for row in success_rows],
        }
        summary: dict[str, object] = {
            "profile": profile,
            "runs": len(group),
            "successes": len(success_rows),
            "failures": len(group) - len(success_rows),
        }
        for key, values in metrics.items():
            mean_value, std_value = mean_std(values)
            summary[f"{key}_mean"] = mean_value
            summary[f"{key}_std"] = std_value
        summaries.append(summary)
    return summaries


def run_trajectory_repeats(args: argparse.Namespace, run_root: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    per_run_rows: list[dict[str, object]] = []
    logs_root = run_root / "trajectory_repeats"
    logs_root.mkdir(parents=True, exist_ok=True)
    env = acceptance_env()

    for repeat in range(1, args.trajectory_repeats + 1):
        repeat_root = logs_root / f"repeat_{repeat:02d}"
        repeat_root.mkdir(parents=True, exist_ok=True)
        raw_pull_root = repeat_root / "raw_pull"
        for traj_id, name in TRAJECTORIES:
            log_path = repeat_root / f"traj_{traj_id}_{name}.log"
            pull_log = repeat_root / f"traj_{traj_id}_{name}_pull.log"
            print(f"[acceptance] trajectory {traj_id} ({name}) repeat {repeat}/{args.trajectory_repeats}", flush=True)
            before_remote_files: dict[str, int | None] = {}
            try:
                before_remote_files = list_remote_tracking_logs(current_cube_port(), args.baud, traj_id)
            except Exception:
                before_remote_files = {}
            proc = run_cmd(
                [str(TRAJECTORY_SCRIPT), str(Path(args.px4_root).expanduser().resolve()), str(traj_id), args.endpoint, str(args.baud)],
                log_path=log_path,
                check=False,
                env=env,
            )
            log_text = log_path.read_text(encoding="utf-8", errors="replace")
            row: dict[str, object] = {
                "repeat": repeat,
                "traj_id": traj_id,
                "name": name,
                "return_code": proc.returncode,
                "success": proc.returncode == 0,
                "log_path": str(log_path),
                "pull_log": str(pull_log),
            }
            try:
                row.update(parse_trajectory_summary(log_text))
            except Exception as exc:
                row["parse_error"] = str(exc)

            try:
                port = wait_for_cube_port()
                after_remote_files = list_remote_tracking_logs(port, args.baud, traj_id)
                latest_csv = pull_tracking_log(
                    port,
                    args.baud,
                    raw_pull_root,
                    traj_id,
                    before_remote_files,
                    after_remote_files,
                )
                rmse, max_err, final_err, rows, duration_s = compute_rmse(latest_csv)
                row["tracking_csv"] = str(latest_csv)
                row["rmse_m"] = rmse
                row["max_err_m"] = max_err
                row["final_err_m"] = final_err
                row["csv_rows"] = rows
                row["csv_duration_s"] = duration_s
            except Exception as exc:
                row["pull_error"] = str(exc)
                print(f"[acceptance] trajectory {traj_id} ({name}) pull failed: {exc}", flush=True)
            per_run_rows.append(row)

            time.sleep(2.0)

    summaries = summarize_trajectory_rows(per_run_rows)
    return per_run_rows, summaries


def best_manifest_entry(manifest: list[dict[str, object]], profile: str) -> dict[str, object] | None:
    profile_entries = [entry for entry in manifest if entry.get("profile") == profile]
    if not profile_entries:
        return None
    success_entries = [entry for entry in profile_entries if entry.get("success") is True]
    if success_entries:
        return success_entries[-1]
    return profile_entries[-1]


def run_ident_repeats(args: argparse.Namespace, run_root: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    profiles = campaign_ident_profiles("identification_only")
    per_run_rows: list[dict[str, object]] = []
    logs_root = run_root / "ident_repeats"
    logs_root.mkdir(parents=True, exist_ok=True)
    env = acceptance_env()

    for repeat in range(1, args.ident_repeats + 1):
        repeat_root = logs_root / f"repeat_{repeat:02d}"
        suite_log = repeat_root / "suite.log"
        print(f"[acceptance] identification suite repeat {repeat}/{args.ident_repeats}", flush=True)
        proc = run_cmd(
            [
                sys.executable,
                str(IDENT_SUITE_SCRIPT),
                "--px4-root",
                str(Path(args.px4_root).expanduser().resolve()),
                "--endpoint",
                args.endpoint,
                "--baud",
                str(args.baud),
                "--run-root",
                str(repeat_root),
                "--max-attempts",
                "1",
            ],
            log_path=suite_log,
            check=False,
            env=env,
        )

        manifest_path = repeat_root / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else []
        for profile in profiles:
            entry = best_manifest_entry(manifest, profile)
            row: dict[str, object] = {
                "repeat": repeat,
                "profile": profile,
                "suite_return_code": proc.returncode,
                "success": False,
                "suite_log": str(suite_log),
            }
            if entry is None:
                row["parse_error"] = "profile missing from manifest"
                per_run_rows.append(row)
                continue

            row["helper_log"] = entry.get("helper_log", "")
            row["ident_csv"] = entry.get("ident_csv", "")
            row["rows"] = entry.get("rows", 0)
            row["duration_s"] = entry.get("duration_s", 0.0)
            row["ref_motion_span_m"] = entry.get("ref_motion_span_m", 0.0)
            row["pos_motion_span_m"] = entry.get("pos_motion_span_m", 0.0)
            row["capture_healthy"] = entry.get("capture_healthy", False)
            row["return_code"] = entry.get("return_code", proc.returncode)
            row["success"] = bool(entry.get("success", False))

            summary_line = str(entry.get("summary_line", ""))
            if summary_line:
                try:
                    row.update(parse_ident_summary(summary_line))
                except Exception as exc:
                    row["parse_error"] = str(exc)
            else:
                row["parse_error"] = "summary line missing"
            per_run_rows.append(row)

    summaries = summarize_ident_rows(per_run_rows, profiles)
    return per_run_rows, summaries


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    run_root.mkdir(parents=True, exist_ok=True)

    trajectory_rows: list[dict[str, object]] = []
    trajectory_summary: list[dict[str, object]] = []
    ident_rows: list[dict[str, object]] = []
    ident_summary: list[dict[str, object]] = []

    try:
        if not args.skip_trajectories:
            trajectory_rows, trajectory_summary = run_trajectory_repeats(args, run_root)
            write_csv(
                run_root / "trajectory_runs.csv",
                trajectory_rows,
                [
                    "repeat", "traj_id", "name", "return_code", "success", "log_path",
                    "tracking_csv", "pull_log", "rmse_m", "max_err_m", "final_err_m", "csv_rows", "csv_duration_s",
                    "max_xy_err", "max_z_err", "max_tilt", "final_xy", "final_z_err", "final_tilt", "failsafe", "parse_error", "pull_error",
                ],
            )
            write_csv(
                run_root / "trajectory_summary.csv",
                trajectory_summary,
                [
                    "traj_id", "name", "runs", "successes", "failures",
                    "max_xy_err_mean", "max_xy_err_std",
                    "max_z_err_mean", "max_z_err_std",
                    "max_tilt_mean", "max_tilt_std",
                    "final_xy_mean", "final_xy_std",
                    "final_z_err_mean", "final_z_err_std",
                    "final_tilt_mean", "final_tilt_std",
                    "rmse_m_mean", "rmse_m_std",
                    "max_err_m_mean", "max_err_m_std",
                    "csv_duration_s_mean", "csv_duration_s_std",
                ],
            )
            write_markdown_table(
                run_root / "trajectory_summary.md",
                ["traj_id", "name", "successes/runs", "rmse mean±std", "max_xy_err mean±std", "max_z_err mean±std", "max_tilt mean±std"],
                [
                    [
                        row["traj_id"],
                        row["name"],
                        f"{row['successes']}/{row['runs']}",
                        f"{row['rmse_m_mean']:.3f} ± {row['rmse_m_std']:.3f}",
                        f"{row['max_xy_err_mean']:.3f} ± {row['max_xy_err_std']:.3f}",
                        f"{row['max_z_err_mean']:.3f} ± {row['max_z_err_std']:.3f}",
                        f"{row['max_tilt_mean']:.2f} ± {row['max_tilt_std']:.2f}",
                    ]
                    for row in trajectory_summary
                ],
            )

        if not args.skip_ident:
            ident_rows, ident_summary = run_ident_repeats(args, run_root)
            write_csv(
                run_root / "ident_runs.csv",
                ident_rows,
                [
                    "repeat", "profile", "suite_return_code", "return_code", "success", "capture_healthy",
                    "rows", "duration_s", "ref_motion_span_m", "pos_motion_span_m",
                    "max_xy", "max_z_err", "max_tilt", "final_xy", "final_z_err", "final_tilt", "failsafe",
                    "suite_log", "helper_log", "ident_csv", "parse_error",
                ],
            )
            write_csv(
                run_root / "ident_summary.csv",
                ident_summary,
                [
                    "profile", "runs", "successes", "failures",
                    "max_xy_mean", "max_xy_std",
                    "max_z_err_mean", "max_z_err_std",
                    "max_tilt_mean", "max_tilt_std",
                    "rows_mean", "rows_std",
                    "duration_s_mean", "duration_s_std",
                    "ref_motion_span_m_mean", "ref_motion_span_m_std",
                    "pos_motion_span_m_mean", "pos_motion_span_m_std",
                ],
            )
            write_markdown_table(
                run_root / "ident_summary.md",
                ["profile", "successes/runs", "max_xy mean±std", "max_z_err mean±std", "max_tilt mean±std", "duration mean±std"],
                [
                    [
                        row["profile"],
                        f"{row['successes']}/{row['runs']}",
                        f"{row['max_xy_mean']:.3f} ± {row['max_xy_std']:.3f}",
                        f"{row['max_z_err_mean']:.3f} ± {row['max_z_err_std']:.3f}",
                        f"{row['max_tilt_mean']:.2f} ± {row['max_tilt_std']:.2f}",
                        f"{row['duration_s_mean']:.2f} ± {row['duration_s_std']:.2f}",
                    ]
                    for row in ident_summary
                ],
            )

        summary_payload = {
            "run_root": str(run_root),
            "trajectory_summary": trajectory_summary,
            "ident_summary": ident_summary,
        }
        (run_root / "summary.json").write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")

        review_dir = run_root / "review"
        if (run_root / "trajectory_repeats").exists():
            run_cmd(
                [
                    sys.executable,
                    str(REVIEW_BUNDLE_SCRIPT),
                    "--log-root",
                    str(run_root / "trajectory_repeats"),
                    "--out-dir",
                    str(review_dir / "trajectories"),
                    "--max-points",
                    "2500",
                ],
                check=True,
            )
        if (run_root / "ident_repeats").exists():
            run_cmd(
                [
                    sys.executable,
                    str(REVIEW_BUNDLE_SCRIPT),
                    "--log-root",
                    str(run_root / "ident_repeats"),
                    "--out-dir",
                    str(review_dir / "ident"),
                    "--max-points",
                    "2500",
                ],
                check=True,
            )
        run_cmd(
            [
                sys.executable,
                str(SUMMARY_FIGURES_SCRIPT),
                "--run-root",
                str(run_root),
                "--out-dir",
                str(run_root / "figures"),
            ],
            check=True,
        )
        return 0

    finally:
        try:
            graceful_stop_hitl_session(args.endpoint)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
