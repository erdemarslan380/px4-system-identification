#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (SCRIPT_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from experimental_validation.hitl_catalog import campaign_expected_duration_s, campaign_ident_profiles
from pull_sdcard_logs_over_mavftp import (
    collect_remote_files,
    download_file_via_port,
)
from run_hitl_ident_suite_single_session import (
    list_remote_ident_logs_with_retry,
    normalize_downloaded_ident_logs,
)
from run_hitl_trajectory_suite_only import (
    current_cube_port,
    graceful_stop_hitl_session,
    run_cmd,
    wait_for_cube_port,
)

RESTART_SCRIPT = SCRIPT_DIR / "restart_hitl_px4_clean_gui.sh"
USB_STREAM_SCRIPT = SCRIPT_DIR / "set_hitl_usb_actuator_stream.sh"
SESSION_HELPER = SCRIPT_DIR / "run_hitl_full_stack_single_session.py"
CAMPAIGN_NAME = "identification_plus_lemniscate_calibration"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Run the full identification maneuver pack plus a final lemniscate calibration trajectory in one visible HIL flight."
    )
    ap.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--run-root", default=str(REPO_ROOT / "hitl_runs" / "ident_plus_lemniscate_calibration"))
    ap.add_argument("--usb-stream-rate", type=int, default=200)
    ap.add_argument("--hover-z", type=float, default=-5.0)
    ap.add_argument("--between-segments-hold-seconds", type=float, default=1.5)
    ap.add_argument("--ident-tail-seconds", type=float, default=4.0)
    ap.add_argument("--trajectory-tail-seconds", type=float, default=6.0)
    ap.add_argument("--custom-hold-seconds", type=float, default=4.0)
    ap.add_argument("--direct-settle-seconds", type=float, default=2.0)
    ap.add_argument("--direct-z-tolerance", type=float, default=0.8)
    return ap.parse_args()


def list_remote_tracking_logs(port: str, baud: int) -> dict[str, int | None]:
    grouped = collect_remote_files(
        port,
        baud,
        10.0,
        {"tracking_logs": "/fs/microsd/tracking_logs"},
        (".csv",),
    )
    return {name: size for name, size in grouped.get("tracking_logs", [])}


def list_remote_tracking_logs_with_retry(port: str, baud: int, attempts: int = 5, delay_s: float = 2.0) -> dict[str, int | None]:
    last_error: Exception | None = None
    for _ in range(max(1, attempts)):
        try:
            return list_remote_tracking_logs(port, baud)
        except Exception as exc:
            last_error = exc
            time.sleep(delay_s)
    raise RuntimeError(f"Unable to list remote tracking logs over MAVFTP: {last_error}")


def download_created_csvs(
    *,
    port: str,
    baud: int,
    before_files: dict[str, int | None],
    after_files: dict[str, int | None],
    remote_dir: str,
    destination_dir: Path,
) -> list[Path]:
    created = sorted(
        name
        for name, size in after_files.items()
        if name not in before_files or before_files.get(name) != size
    )
    destination_dir.mkdir(parents=True, exist_ok=True)
    downloaded: list[Path] = []
    for name in created:
        local_path = destination_dir / name
        downloaded_path = download_file_via_port(
            port=port,
            baud=baud,
            heartbeat_timeout=10.0,
            remote_path=f"{remote_dir}/{name}",
            local_path=local_path,
            expected_size=after_files[name],
            retries=2,
            timeout=30.0,
        )
        downloaded.append(downloaded_path)
    return downloaded


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    console_dir = run_root / "console_logs"
    raw_pull_root = run_root / "raw_pull"
    ident_pull_root = raw_pull_root / "identification_logs"
    tracking_pull_root = raw_pull_root / "tracking_logs"
    summary_json = run_root / "single_session_summary.json"
    manifest_path = run_root / "manifest.json"
    run_root.mkdir(parents=True, exist_ok=True)
    console_dir.mkdir(parents=True, exist_ok=True)

    restart_log = console_dir / "restart.log"
    stream_log = console_dir / "usb_stream.log"
    helper_log = console_dir / "single_session_helper.log"

    before_ident_files: dict[str, int | None] = {}
    before_tracking_files: dict[str, int | None] = {}
    try:
        port = wait_for_cube_port(timeout_s=30.0)
        before_ident_files = list_remote_ident_logs_with_retry(port, args.baud)
        before_tracking_files = list_remote_tracking_logs_with_retry(port, args.baud)
    except Exception:
        before_ident_files = {}
        before_tracking_files = {}

    run_cmd(
        [str(RESTART_SCRIPT), args.px4_root, str(args.baud), args.endpoint],
        log_path=restart_log,
        check=True,
        env=os.environ.copy(),
    )

    run_cmd(
        [str(USB_STREAM_SCRIPT), str(args.usb_stream_rate), args.endpoint],
        log_path=stream_log,
        check=False,
    )

    run_cmd(
        [
            sys.executable,
            str(SESSION_HELPER),
            "--campaign",
            CAMPAIGN_NAME,
            "--endpoint",
            args.endpoint,
            "--baud",
            str(args.baud),
            "--hover-z",
            str(args.hover_z),
            "--between-segments-hold-seconds",
            str(args.between_segments_hold_seconds),
            "--ident-tail-seconds",
            str(args.ident_tail_seconds),
            "--trajectory-tail-seconds",
            str(args.trajectory_tail_seconds),
            "--custom-hold-seconds",
            str(args.custom_hold_seconds),
            "--direct-settle-seconds",
            str(args.direct_settle_seconds),
            "--direct-z-tolerance",
            str(args.direct_z_tolerance),
            "--output-json",
            str(summary_json),
        ],
        log_path=helper_log,
        check=True,
    )

    graceful_stop_hitl_session(args.endpoint)
    time.sleep(2.0)
    port = wait_for_cube_port(timeout_s=90.0)

    after_ident_files = list_remote_ident_logs_with_retry(port, args.baud)
    downloaded_ident = download_created_csvs(
        port=port,
        baud=args.baud,
        before_files=before_ident_files,
        after_files=after_ident_files,
        remote_dir="/fs/microsd/identification_logs",
        destination_dir=ident_pull_root,
    )
    downloaded_ident = normalize_downloaded_ident_logs(downloaded_ident, campaign_ident_profiles(CAMPAIGN_NAME))

    after_tracking_files = list_remote_tracking_logs_with_retry(port, args.baud)
    downloaded_tracking = download_created_csvs(
        port=port,
        baud=args.baud,
        before_files=before_tracking_files,
        after_files=after_tracking_files,
        remote_dir="/fs/microsd/tracking_logs",
        destination_dir=tracking_pull_root,
    )

    lemniscate_tracking = sorted(
        [path for path in downloaded_tracking if path.name.startswith("t101") or "lemniscate" in path.name.lower()],
        key=lambda path: path.name,
    )

    payload = {
        "campaign": CAMPAIGN_NAME,
        "expected_duration_s": campaign_expected_duration_s(CAMPAIGN_NAME),
        "summary_json": str(summary_json),
        "restart_log": str(restart_log),
        "usb_stream_log": str(stream_log),
        "helper_log": str(helper_log),
        "identification_logs": [str(path.resolve()) for path in downloaded_ident],
        "tracking_logs": [str(path.resolve()) for path in downloaded_tracking],
        "lemniscate_tracking_logs": [str(path.resolve()) for path in lemniscate_tracking],
    }
    manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
