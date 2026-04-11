#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (SCRIPT_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from experimental_validation.hitl_catalog import campaign_ident_profiles
from pull_sdcard_logs_over_mavftp import (
    collect_remote_files,
    connect_mavftp,
    download_file_via_port,
    list_remote_entries,
)
from pymavlink.mavftp import FtpError
from run_hitl_trajectory_suite_only import (
    current_cube_port,
    graceful_stop_hitl_session,
    run_cmd,
    wait_for_cube_port,
)

RESTART_SCRIPT = SCRIPT_DIR / "restart_hitl_px4_clean_gui.sh"
USB_STREAM_SCRIPT = SCRIPT_DIR / "set_hitl_usb_actuator_stream.sh"
SINGLE_SESSION_HELPER = SCRIPT_DIR / "run_hitl_px4_builtin_ident_sequence.py"
TRUTH_SPLITTER = REPO_ROOT / "experimental_validation" / "split_jmavsim_truth_session.py"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run all built-in identification maneuvers in a single visible HIL flight.")
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--run-root", default=str(REPO_ROOT / "hitl_runs" / "ident_suite_single_session"))
    ap.add_argument("--usb-stream-rate", type=int, default=200)
    ap.add_argument("--hover-z", type=float, default=-5.0)
    ap.add_argument("--profiles", nargs="*", default=campaign_ident_profiles("identification_only"))
    return ap.parse_args()


def list_remote_ident_logs(port: str, baud: int) -> dict[str, int | None]:
    grouped = collect_remote_files(
        port,
        baud,
        10.0,
        {"identification_logs": "/fs/microsd/identification_logs"},
        (".csv",),
    )
    return {name: size for name, size in grouped.get("identification_logs", [])}


def list_remote_ident_logs_with_retry(port: str, baud: int, attempts: int = 5, delay_s: float = 2.0) -> dict[str, int | None]:
    last_error: Exception | None = None
    for _ in range(max(1, attempts)):
        try:
            return list_remote_ident_logs(port, baud)
        except Exception as exc:
            last_error = exc
            time.sleep(delay_s)
    raise RuntimeError(f"Unable to list remote identification logs over MAVFTP: {last_error}")


def clear_remote_sdcard_logs(port: str, baud: int, *, log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    remote_dirs = [
        "/fs/microsd/tracking_logs",
        "/fs/microsd/identification_logs",
    ]

    master, ftp = connect_mavftp(port, baud, heartbeat_timeout=20.0)
    try:
        with log_path.open("w", encoding="utf-8") as log_file:
            for remote_dir in remote_dirs:
                entries = [
                    entry for entry in list_remote_entries(ftp, remote_dir)
                    if not getattr(entry, "is_dir", False) and getattr(entry, "name", "").endswith(".csv")
                ]
                log_file.write(f"[scan] {remote_dir}: {len(entries)} csv files\n")
                for entry in entries:
                    remote_path = f"{remote_dir}/{entry.name}"
                    result = ftp.cmd_rm([remote_path])
                    log_file.write(
                        f"[rm] {remote_path}: return_code={result.return_code}\n"
                    )
                    if result.return_code not in {FtpError.Success, FtpError.FileNotFound}:
                        raise RuntimeError(
                            f"Failed to remove remote log {remote_path}: return_code={result.return_code}"
                        )

            for remote_dir in remote_dirs:
                remaining = [
                    entry.name for entry in list_remote_entries(ftp, remote_dir)
                    if not getattr(entry, "is_dir", False) and getattr(entry, "name", "").endswith(".csv")
                ]
                log_file.write(f"[verify] {remote_dir}: {len(remaining)} csv files remain\n")
                if remaining:
                    raise RuntimeError(
                        f"Remote log cleanup incomplete for {remote_dir}: {remaining[:5]}"
                    )
    finally:
        master.close()


def _sanitize_identification_csv(path: Path) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return
    header = lines[0]
    cleaned = [header]
    for line in lines[1:]:
        if not line.strip():
            continue
        if line.startswith(("INFO ", "WARN ", "ERROR ")):
            continue
        if not line[:1].isdigit():
            continue
        cleaned.append(line)
    if cleaned != lines:
        path.write_text("\n".join(cleaned) + "\n", encoding="utf-8")


def _run_counter_from_name(path: Path) -> int:
    match = re.search(r"_r(\d+)_", path.name)
    return int(match.group(1)) if match else 0


def normalize_downloaded_ident_logs(downloaded: list[Path], expected_profiles: list[str]) -> list[Path]:
    cleaned: list[Path] = []
    ordered = sorted(downloaded, key=_run_counter_from_name)
    for path in ordered:
        _sanitize_identification_csv(path)
        cleaned.append(path)

    if len(ordered) != len(expected_profiles):
        return cleaned

    aliased: list[Path] = []
    for path, expected_profile in zip(ordered, expected_profiles):
        if path.name.startswith(f"{expected_profile}_"):
            aliased.append(path)
            continue
        suffix = path.name.split("_r", 1)[1] if "_r" in path.name else path.name
        alias = path.with_name(f"{expected_profile}_r{suffix}")
        shutil.copy2(path, alias)
        _sanitize_identification_csv(alias)
        aliased.append(alias)
    return aliased


def download_created_ident_logs(
    *,
    port: str,
    baud: int,
    before_files: dict[str, int | None],
    destination_dir: Path,
    expected_profiles: list[str],
) -> list[Path]:
    after_files = list_remote_ident_logs_with_retry(port, baud)
    created = sorted(
        name
        for name, size in after_files.items()
        if name not in before_files or before_files.get(name) != size
    )
    if not created:
        raise RuntimeError("No new identification CSVs appeared on the SD card")

    destination_dir.mkdir(parents=True, exist_ok=True)
    downloaded: list[Path] = []
    for name in created:
        local_path = destination_dir / name
        downloaded_path = download_file_via_port(
            port=port,
            baud=baud,
            heartbeat_timeout=10.0,
            remote_path=f"/fs/microsd/identification_logs/{name}",
            local_path=local_path,
            expected_size=after_files[name],
            retries=2,
            timeout=30.0,
        )
        downloaded.append(downloaded_path)
    return normalize_downloaded_ident_logs(downloaded, expected_profiles)


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    console_dir = run_root / "console_logs"
    raw_pull_root = run_root / "raw_pull"
    ident_pull_root = raw_pull_root / "identification_logs"
    truth_pull_root = raw_pull_root / "jmavsim_truth"
    truth_session_csv = truth_pull_root / "full_session_truth.csv"
    summary_json = run_root / "single_session_summary.json"
    manifest_path = run_root / "manifest.json"
    run_root.mkdir(parents=True, exist_ok=True)
    console_dir.mkdir(parents=True, exist_ok=True)
    truth_pull_root.mkdir(parents=True, exist_ok=True)
    if truth_session_csv.exists():
        truth_session_csv.unlink()

    restart_log = console_dir / "restart.log"
    stream_log = console_dir / "usb_stream.log"
    helper_log = console_dir / "single_session_helper.log"
    sdcard_cleanup_log = console_dir / "sdcard_cleanup.log"

    before_remote_files: dict[str, int | None] = {}
    try:
        port = wait_for_cube_port(timeout_s=30.0)
        before_remote_files = list_remote_ident_logs_with_retry(port, args.baud)
        sdcard_cleanup_log.write_text(
            "Skipped destructive SD cleanup: unique identification log session IDs are enabled.\n",
            encoding="utf-8",
        )
    except Exception:
        before_remote_files = {}

    restart_env = os.environ.copy()
    restart_env["JMAVSIM_TRUTH_LOG"] = str(truth_session_csv)
    restart_env["JMAVSIM_TRUTH_LOG_INTERVAL_US"] = "20000"
    run_cmd(
        [str(RESTART_SCRIPT), args.px4_root, str(args.baud), args.endpoint],
        log_path=restart_log,
        check=True,
        env=restart_env,
    )

    run_cmd(
        [str(USB_STREAM_SCRIPT), str(args.usb_stream_rate), args.endpoint],
        log_path=stream_log,
        check=False,
    )

    run_cmd(
        [
            sys.executable,
            str(SINGLE_SESSION_HELPER),
            "--endpoint",
            args.endpoint,
            "--baud",
            str(args.baud),
            "--hover-z",
            str(args.hover_z),
            "--profiles",
            *args.profiles,
            "--output-json",
            str(summary_json),
        ],
        log_path=helper_log,
        check=True,
    )

    graceful_stop_hitl_session(args.endpoint)
    time.sleep(2.0)
    port = wait_for_cube_port(timeout_s=90.0)
    downloaded = download_created_ident_logs(
        port=port,
        baud=args.baud,
        before_files=before_remote_files,
        destination_dir=ident_pull_root,
        expected_profiles=list(args.profiles),
    )

    split_truth_log = console_dir / "split_truth.log"
    truth_files: list[Path] = []
    if truth_session_csv.exists():
        run_cmd(
            [
                sys.executable,
                str(TRUTH_SPLITTER),
                "--ident-root",
                str(ident_pull_root),
                "--truth-csv",
                str(truth_session_csv),
                "--summary-json",
                str(summary_json),
                "--out-dir",
                str(truth_pull_root),
            ],
            log_path=split_truth_log,
            check=True,
        )
        truth_files = sorted(
            [path for path in truth_pull_root.glob("*.csv") if path.name != truth_session_csv.name],
            key=lambda path: path.name,
        )
    payload = {
        "profiles": list(args.profiles),
        "summary_json": str(summary_json),
        "restart_log": str(restart_log),
        "usb_stream_log": str(stream_log),
        "helper_log": str(helper_log),
        "sdcard_cleanup_log": str(sdcard_cleanup_log),
        "split_truth_log": str(split_truth_log),
        "identification_logs": [str(path.resolve()) for path in downloaded],
        "full_session_truth_log": str(truth_session_csv.resolve()) if truth_session_csv.exists() else "",
        "truth_logs": [str(path.resolve()) for path in truth_files],
    }
    manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
