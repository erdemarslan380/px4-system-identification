#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path

import fcntl

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP


REMOTE_GROUPS = {
    "tracking_logs": "/fs/microsd/tracking_logs",
    "identification_logs": "/fs/microsd/identification_logs",
}

KNOWN_USB_CDC_HOLDER_PATTERNS = [
    "jmavsim_run.jar",
    "Tools/mavlink_shell.py",
    "examples/hitl_shell_smoke.py",
    "examples/px4_nsh_runner.py",
]


def port_lock_name(port: str) -> Path:
    return Path("/tmp") / f"px4_mavftp_{Path(port).name}.lock"


@contextmanager
def exclusive_port_lock(port: str):
    lock_path = port_lock_name(port)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("w", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"Another MAVFTP pull is already using {port}") from exc
        handle.write(f"{os.getpid()}\n")
        handle.flush()
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def wait_for_heartbeat(master, timeout: float):
    deadline = time.time() + timeout
    while time.time() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None:
            return heartbeat
    return None


def connect_mavftp(port: str, baud: int, heartbeat_timeout: float) -> tuple[object, MAVFTP]:
    last_error: Exception | None = None
    for _attempt in range(3):
        master = None
        try:
            master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
            heartbeat = wait_for_heartbeat(master, timeout=heartbeat_timeout)
            if heartbeat is None:
                raise RuntimeError(f"No MAVLink heartbeat received on {port} @ {baud}")
            ftp = MAVFTP(master, master.target_system, master.target_component)
            return master, ftp
        except Exception as exc:  # pragma: no cover - exercised on hardware
            last_error = exc
            if master is not None:
                master.close()
            time.sleep(0.5)
    raise RuntimeError(f"Unable to establish MAVFTP over {port}: {last_error}")


def list_port_holders(port: str) -> list[int]:
    result = subprocess.run(
        ["lsof", "-t", port],
        capture_output=True,
        text=True,
        check=False,
    )
    pids: list[int] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.isdigit():
            pids.append(int(line))
    return sorted(set(pids))


def cleanup_known_holders(current_pid: int | None = None) -> None:
    for pattern in KNOWN_USB_CDC_HOLDER_PATTERNS:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True,
            text=True,
            check=False,
        )
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line.isdigit():
                continue
            pid = int(line)
            if current_pid is not None and pid == current_pid:
                continue
            subprocess.run(["kill", "-9", str(pid)], check=False)


def wait_for_port_free(port: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not list_port_holders(port):
            return True
        time.sleep(0.2)
    return not list_port_holders(port)


def local_file_matches(local_path: Path, expected_size: int | None) -> bool:
    if not local_path.exists():
        return False
    if expected_size is None:
        return local_path.stat().st_size > 0
    return local_path.stat().st_size == expected_size


def fresh_ftp_client(ftp: MAVFTP) -> MAVFTP:
    master = getattr(ftp, "master", None)
    target_system = getattr(ftp, "target_system", None)
    target_component = getattr(ftp, "target_component", None)
    ftp_settings = getattr(ftp, "ftp_settings", None)

    if master is None or target_system is None or target_component is None:
        return ftp

    if ftp_settings is not None:
        return ftp.__class__(master, target_system, target_component, settings=ftp_settings)

    return ftp.__class__(master, target_system, target_component)


def list_remote_entries(ftp: MAVFTP, remote_dir: str) -> list[object]:
    ret = ftp.cmd_list([remote_dir])

    if ret.return_code != FtpError.Success:
        return []

    return list(ftp.list_result)


def list_remote_matching_files(
    ftp: MAVFTP,
    remote_dir: str,
    suffixes: tuple[str, ...] = (".csv",),
) -> list[str]:
    matching_names = []

    for entry in list_remote_entries(ftp, remote_dir):
        if entry.is_dir:
            continue

        if entry.name.endswith(suffixes):
            matching_names.append(entry.name)

    return sorted(matching_names)


def list_remote_csvs(ftp: MAVFTP, remote_dir: str) -> list[str]:
    return list_remote_matching_files(ftp, remote_dir, suffixes=(".csv",))


def expected_size_from_entry(entry: object) -> int | None:
    size = getattr(entry, "size_b", None)
    if isinstance(size, int) and size >= 0:
        return size
    return None


def ftp_error_label(return_code: int) -> str:
    for name, value in FtpError.__dict__.items():
        if isinstance(value, int) and value == return_code:
            return name
    return f"code_{return_code}"


def download_file_with_retries(
    ftp: MAVFTP,
    remote_path: str,
    local_path: Path,
    expected_size: int | None,
    retries: int,
    timeout: float,
) -> Path:
    attempts = 0
    last_error: Exception | None = None

    while attempts <= retries:
        attempts += 1
        active_ftp = fresh_ftp_client(ftp)
        local_path.parent.mkdir(parents=True, exist_ok=True)

        if local_path.exists():
            local_path.unlink()

        active_ftp.cmd_get([remote_path, str(local_path)])
        result = active_ftp.process_ftp_reply("OpenFileRO", timeout=timeout)

        if result.return_code == FtpError.Success and local_path.exists():
            if expected_size is None or local_path.stat().st_size == expected_size:
                return local_path
            last_error = RuntimeError(
                f"Downloaded size mismatch for {remote_path}: "
                f"expected {expected_size}, got {local_path.stat().st_size}"
            )
        else:
            last_error = RuntimeError(
                f"Download failed for {remote_path}: "
                f"return_code={result.return_code} ({ftp_error_label(result.return_code)})"
            )

        if local_path.exists():
            local_path.unlink()

        time.sleep(0.3)

    raise RuntimeError(str(last_error) if last_error else f"Download failed for {remote_path}")


def collect_remote_files(
    port: str,
    baud: int,
    heartbeat_timeout: float,
    remote_groups: dict[str, str],
    suffixes: tuple[str, ...],
) -> dict[str, list[tuple[str, int | None]]]:
    master, ftp = connect_mavftp(port, baud, heartbeat_timeout)
    try:
        grouped: dict[str, list[tuple[str, int | None]]] = {}
        for local_group, remote_dir in remote_groups.items():
            files: list[tuple[str, int | None]] = []
            for entry in list_remote_entries(ftp, remote_dir):
                if entry.is_dir or not entry.name.endswith(suffixes):
                    continue
                files.append((entry.name, expected_size_from_entry(entry)))
            grouped[local_group] = files
        return grouped
    finally:
        master.close()


def download_file_via_port(
    *,
    port: str,
    baud: int,
    heartbeat_timeout: float,
    remote_path: str,
    local_path: Path,
    expected_size: int | None,
    retries: int,
    timeout: float,
) -> Path:
    attempts = 0
    last_error: Exception | None = None

    local_path.parent.mkdir(parents=True, exist_ok=True)

    if local_file_matches(local_path, expected_size):
        return local_path

    while attempts <= retries:
        attempts += 1
        master = None
        try:
            master, ftp = connect_mavftp(port, baud, heartbeat_timeout)
            ftp.cmd_get([remote_path, str(local_path)])
            result = ftp.process_ftp_reply("OpenFileRO", timeout=timeout)
            if result.return_code == FtpError.Success and local_path.exists():
                if expected_size is None or local_path.stat().st_size == expected_size:
                    return local_path
                last_error = RuntimeError(
                    f"Downloaded size mismatch for {remote_path}: "
                    f"expected {expected_size}, got {local_path.stat().st_size}"
                )
            else:
                last_error = RuntimeError(
                    f"Download failed for {remote_path}: "
                    f"return_code={result.return_code} ({ftp_error_label(result.return_code)})"
                )
        except Exception as exc:  # pragma: no cover - exercised on hardware
            last_error = exc
        finally:
            if master is not None:
                master.close()

        if local_path.exists():
            local_path.unlink()

        time.sleep(0.5)

    raise RuntimeError(str(last_error) if last_error else f"Download failed for {remote_path}")


def write_pull_report(
    destination_dir: Path,
    *,
    pulled: dict[str, list[str]],
    skipped: dict[str, list[str]],
    failed: dict[str, list[dict[str, str]]],
) -> Path:
    report_path = destination_dir / "pull_report.json"
    payload = {
        "pulled": pulled,
        "skipped": skipped,
        "failed": failed,
    }
    report_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return report_path


def download_logs(
    ftp: MAVFTP,
    destination_dir: Path,
    remote_groups: dict[str, str] | None = None,
    suffixes: tuple[str, ...] = (".csv",),
    retries: int = 2,
    timeout: float = 30.0,
) -> dict[str, list[Path]]:
    remote_groups = remote_groups or REMOTE_GROUPS
    pulled: dict[str, list[Path]] = {}

    destination_dir.mkdir(parents=True, exist_ok=True)

    for local_group, remote_dir in remote_groups.items():
        local_dir = destination_dir / local_group
        local_dir.mkdir(parents=True, exist_ok=True)

        pulled_group: list[Path] = []

        for entry in list_remote_entries(ftp, remote_dir):
            if entry.is_dir or not entry.name.endswith(suffixes):
                continue

            remote_path = f"{remote_dir}/{entry.name}"
            local_path = local_dir / entry.name
            pulled_group.append(
                download_file_with_retries(
                    ftp=ftp,
                    remote_path=remote_path,
                    local_path=local_path,
                    expected_size=expected_size_from_entry(entry),
                    retries=retries,
                    timeout=timeout,
                )
            )

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
        "--remote-group",
        action="append",
        default=[],
        help="Optional group mapping in the form local_name=/fs/microsd/path. Repeat as needed.",
    )
    parser.add_argument(
        "--suffix",
        action="append",
        default=[],
        help="Filename suffix to pull, for example .csv or .ulg. Repeat for multiple suffixes.",
    )
    parser.add_argument(
        "--download-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for each MAVFTP download to complete.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="Retry count per file when MAVFTP download fails or size mismatches.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable verbose logging.",
    )
    parser.add_argument(
        "--cleanup-known-holders",
        action="store_true",
        help="Kill known old jMAVSim/MAVLink helper processes before opening the USB CDC port.",
    )
    parser.add_argument(
        "--port-free-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the USB CDC port to become free.",
    )
    return parser


def parse_remote_groups(raw_groups: list[str]) -> dict[str, str]:
    if not raw_groups:
        return dict(REMOTE_GROUPS)

    parsed: dict[str, str] = {}
    for item in raw_groups:
        if "=" not in item:
            raise ValueError(f"Invalid --remote-group value: {item}")
        local_name, remote_dir = item.split("=", 1)
        local_name = local_name.strip()
        remote_dir = remote_dir.strip()
        if not local_name or not remote_dir:
            raise ValueError(f"Invalid --remote-group value: {item}")
        parsed[local_name] = remote_dir
    return parsed


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO if args.debug else logging.WARNING,
        format="%(message)s",
    )

    with exclusive_port_lock(args.port):
        if args.cleanup_known_holders:
            cleanup_known_holders(current_pid=os.getpid())

        if not wait_for_port_free(args.port, timeout=args.port_free_timeout):
            holders = ", ".join(str(pid) for pid in list_port_holders(args.port))
            raise RuntimeError(f"Port {args.port} is still busy: {holders}")

        remote_groups = parse_remote_groups(args.remote_group)
        suffixes = tuple(args.suffix or [".csv"])
        remote_files = collect_remote_files(
            args.port,
            args.baud,
            args.heartbeat_timeout,
            remote_groups,
            suffixes,
        )

        pulled: dict[str, list[Path]] = {}
        skipped: dict[str, list[Path]] = {}
        failed: dict[str, list[dict[str, str]]] = {}
        args.destination_dir.mkdir(parents=True, exist_ok=True)

        for local_group, files in remote_files.items():
            local_dir = args.destination_dir / local_group
            local_dir.mkdir(parents=True, exist_ok=True)
            pulled_group: list[Path] = []
            skipped_group: list[Path] = []
            failed_group: list[dict[str, str]] = []
            remote_dir = remote_groups[local_group]
            for file_name, expected_size in files:
                remote_path = f"{remote_dir}/{file_name}"
                local_path = local_dir / file_name

                if local_file_matches(local_path, expected_size):
                    print(f"Skipping existing file: {file_name}")
                    skipped_group.append(local_path)
                    continue

                print(f"Pulling {local_group}/{file_name}")
                try:
                    pulled_group.append(
                        download_file_via_port(
                            port=args.port,
                            baud=args.baud,
                            heartbeat_timeout=args.heartbeat_timeout,
                            remote_path=remote_path,
                            local_path=local_path,
                            expected_size=expected_size,
                            retries=args.retries,
                            timeout=args.download_timeout,
                        )
                    )
                    print(f"  OK: {file_name}")
                except Exception as exc:  # pragma: no cover - exercised on hardware
                    print(f"  FAILED: {file_name}: {exc}")
                    failed_group.append({"file": file_name, "error": str(exc)})

            pulled[local_group] = pulled_group
            skipped[local_group] = skipped_group
            failed[local_group] = failed_group

    report_path = write_pull_report(
        args.destination_dir,
        pulled={group: [path.name for path in paths] for group, paths in pulled.items()},
        skipped={group: [path.name for path in paths] for group, paths in skipped.items()},
        failed=failed,
    )

    print(f"Pulled logs into: {args.destination_dir}")
    print(f"Report: {report_path}")

    for group_name in remote_groups:
        print(f"{group_name}:")
        for path in pulled.get(group_name, []):
            print(f"  pulled  {path.name}")
        for path in skipped.get(group_name, []):
            print(f"  skipped {path.name}")
        for item in failed.get(group_name, []):
            print(f"  failed  {item['file']}: {item['error']}")
        if not pulled.get(group_name) and not skipped.get(group_name) and not failed.get(group_name):
            print("  (no matching files found)")

    failed_count = sum(len(items) for items in failed.values())
    return 1 if failed_count else 0


if __name__ == "__main__":
    raise SystemExit(main())
