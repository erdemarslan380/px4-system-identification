#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP


REMOTE_GROUPS = {
    "tracking_logs": "/fs/microsd/tracking_logs",
    "identification_logs": "/fs/microsd/identification_logs",
}


def list_remote_csvs(ftp: MAVFTP, remote_dir: str) -> list[str]:
    ret = ftp.cmd_list([remote_dir])

    if ret.return_code != FtpError.Success:
        return []

    csv_names = []

    for entry in ftp.list_result:
        if entry.is_dir:
            continue

        if entry.name.endswith(".csv"):
            csv_names.append(entry.name)

    return sorted(csv_names)


def download_logs(
    ftp: MAVFTP,
    destination_dir: Path,
    remote_groups: dict[str, str] | None = None,
) -> dict[str, list[Path]]:
    remote_groups = remote_groups or REMOTE_GROUPS
    pulled: dict[str, list[Path]] = {}

    destination_dir.mkdir(parents=True, exist_ok=True)

    for local_group, remote_dir in remote_groups.items():
        local_dir = destination_dir / local_group
        local_dir.mkdir(parents=True, exist_ok=True)

        pulled_group: list[Path] = []

        for csv_name in list_remote_csvs(ftp, remote_dir):
            remote_path = f"{remote_dir}/{csv_name}"
            local_path = local_dir / csv_name
            result = ftp.cmd_get([remote_path, str(local_path)])

            if result is None:
                raise RuntimeError(f"Download failed for {remote_path}")

            pulled_group.append(local_path)

        pulled[local_group] = pulled_group

    return pulled


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pull HITL CSV logs from the CubeOrange SD card over MAVFTP.",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port that carries MAVLink access to the flight controller.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="Baud rate for the MAVLink serial link.",
    )
    parser.add_argument(
        "--destination-dir",
        type=Path,
        required=True,
        help="Local directory where tracking_logs/ and identification_logs/ will be written.",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the initial MAVLink heartbeat.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable verbose logging.",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO if args.debug else logging.WARNING,
        format="%(message)s",
    )

    master = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False)
    heartbeat = master.wait_heartbeat(timeout=args.heartbeat_timeout)

    if heartbeat is None:
        raise RuntimeError(f"No MAVLink heartbeat received on {args.port} @ {args.baud}")

    ftp = MAVFTP(master, master.target_system, master.target_component)
    pulled = download_logs(ftp, args.destination_dir)

    print(f"Pulled logs into: {args.destination_dir}")

    for group_name in ("tracking_logs", "identification_logs"):
        print(f"{group_name}:")
        for path in pulled.get(group_name, []):
            print(f"  {path.name}")
        if not pulled.get(group_name):
            print("  (no csv files found)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
