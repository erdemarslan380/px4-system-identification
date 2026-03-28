#!/usr/bin/env python3

"""Run NSH commands over MAVLink SERIAL_CONTROL and print the responses."""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 57600
PROMPT = "nsh>"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="MAVLink serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="MAVLink baud")
    parser.add_argument(
        "--cmd",
        action="append",
        default=[],
        help="Command to run. Repeat for multiple commands.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=6.0,
        help="Per-command timeout in seconds.",
    )
    parser.add_argument(
        "--quiet-timeout",
        type=float,
        default=0.7,
        help="How long to wait after the last received bytes.",
    )
    parser.add_argument(
        "--wait-heartbeat",
        type=float,
        default=15.0,
        help="Heartbeat wait timeout in seconds.",
    )
    return parser.parse_args()


def connect(port: str, baud: int, heartbeat_timeout: float):
    mav = mavutil.mavlink_connection(port, autoreconnect=False, baud=baud)
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )

    deadline = time.time() + heartbeat_timeout
    while time.time() < deadline:
        heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None:
            return mav

    raise TimeoutError(f"No MAVLink heartbeat received on {port} @ {baud}")
    return mav


def send_command(
    mav,
    command: str,
    timeout: float,
    quiet_timeout: float,
) -> str:
    payload = [ord(ch) for ch in (command + "\n")]
    if len(payload) > 70:
        raise ValueError(f"Command too long for SERIAL_CONTROL: {command}")
    payload.extend([0] * (70 - len(payload)))

    mav.mav.serial_control_send(
        0,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
        0,
        0,
        len(command) + 1,
        payload,
    )

    end_time = time.time() + timeout
    quiet_deadline = None
    chunks: list[str] = []

    while time.time() < end_time:
        message = mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if message is not None and message.count:
            text = "".join(chr(x) for x in message.data[: message.count])
            chunks.append(text)
            quiet_deadline = time.time() + quiet_timeout
            if PROMPT in text:
                break
        elif quiet_deadline is not None and time.time() >= quiet_deadline:
            break

    return "".join(chunks)


def main() -> int:
    args = parse_args()
    if not args.cmd:
        raise SystemExit("At least one --cmd is required.")

    mav = connect(args.port, args.baud, args.wait_heartbeat)
    try:
        for command in args.cmd:
            print(f">>> {command}", flush=True)
            response = send_command(mav, command, args.timeout, args.quiet_timeout)
            print(response if response else "[no output]", flush=True)
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
    finally:
        mav.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
