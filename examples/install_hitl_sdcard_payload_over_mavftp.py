#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from pathlib import Path

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJ_SOURCE = REPO_ROOT / "assets" / "validation_trajectories"
REMOTE_TRAJECTORY_DIR = "/fs/microsd/trajectories"
REMOTE_TRACKING_DIR = "/fs/microsd/tracking_logs"
REMOTE_IDENT_DIR = "/fs/microsd/identification_logs"
TRAJECTORY_IDS = (100, 101, 102, 103, 104)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install validation trajectory payload onto the HITL SD card over MAVFTP."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--heartbeat-timeout", type=float, default=20.0)
    parser.add_argument("--source-dir", default=str(DEFAULT_TRAJ_SOURCE))
    return parser.parse_args()


def wait_heartbeat(master, timeout: float):
    deadline = time.time() + timeout
    while time.time() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None:
            return heartbeat
    raise TimeoutError(f"No heartbeat on {master.address} within {timeout:.1f}s")


def connect_ftp(port: str, baud: int, heartbeat_timeout: float) -> tuple[object, MAVFTP]:
    master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
    wait_heartbeat(master, heartbeat_timeout)
    ftp = MAVFTP(master, master.target_system, master.target_component)
    return master, ftp


def fresh_ftp(master) -> MAVFTP:
    return MAVFTP(master, master.target_system, master.target_component)


def list_entries(ftp: MAVFTP, remote_dir: str) -> list[object]:
    ret = ftp.cmd_list([remote_dir])
    if ret.return_code != FtpError.Success:
        return []
    return list(ftp.list_result)


def ensure_directory(ftp: MAVFTP, remote_dir: str) -> None:
    if list_entries(ftp, remote_dir):
        return
    ret = ftp.cmd_mkdir([remote_dir])
    if ret.return_code != FtpError.Success:
        retry_entries = list_entries(ftp, remote_dir)
        if retry_entries or remote_dir in (REMOTE_TRACKING_DIR, REMOTE_IDENT_DIR, REMOTE_TRAJECTORY_DIR):
            return
        raise RuntimeError(f"Failed to create remote directory {remote_dir}: {ret.return_code}")


def remote_file_size(ftp: MAVFTP, remote_dir: str, name: str) -> int | None:
    for entry in list_entries(ftp, remote_dir):
        if not entry.is_dir and entry.name == name:
            size = getattr(entry, "size_b", None)
            if isinstance(size, int):
                return size
    return None


def upload_file(ftp: MAVFTP, local_path: Path, remote_path: str) -> None:
    try:
        ftp.cmd_rm([remote_path])
    except Exception:
        pass

    start_ret = ftp.cmd_put([str(local_path), remote_path])
    if start_ret.return_code != FtpError.Success:
        raise RuntimeError(f"Failed to start upload for {local_path.name}: {start_ret.return_code}")

    result = ftp.process_ftp_reply("CreateFile", timeout=120)
    if result.return_code != FtpError.Success:
        raise RuntimeError(f"Upload failed for {local_path.name}: {result.return_code}")


def main() -> int:
    args = parse_args()
    source_dir = Path(args.source_dir).expanduser().resolve()
    if not source_dir.is_dir():
        raise SystemExit(f"Trajectory source directory not found: {source_dir}")

    required_files = [source_dir / f"id_{traj_id}.traj" for traj_id in TRAJECTORY_IDS]
    missing = [str(path) for path in required_files if not path.is_file()]
    if missing:
        raise SystemExit("Missing trajectory assets:\n" + "\n".join(missing))

    master, ftp = connect_ftp(args.port, args.baud, args.heartbeat_timeout)
    try:
        ensure_directory(ftp, REMOTE_TRAJECTORY_DIR)
        ensure_directory(ftp, REMOTE_TRACKING_DIR)
        ensure_directory(ftp, REMOTE_IDENT_DIR)

        for local_path in required_files:
            remote_path = f"{REMOTE_TRAJECTORY_DIR}/{local_path.name}"
            print(f"Uploading {local_path.name} -> {remote_path}", flush=True)
            upload_file(fresh_ftp(master), local_path, remote_path)
            size = remote_file_size(fresh_ftp(master), REMOTE_TRAJECTORY_DIR, local_path.name)
            if size != local_path.stat().st_size:
                raise RuntimeError(
                    f"Remote size mismatch for {local_path.name}: expected {local_path.stat().st_size}, got {size}"
                )

        print("SD card payload is ready.", flush=True)
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
