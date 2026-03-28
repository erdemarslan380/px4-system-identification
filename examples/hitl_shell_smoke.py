#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

from pymavlink import mavutil


ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


class MavlinkShell:
    def __init__(self, port: str, baudrate: int) -> None:
        self._mav = mavutil.mavlink_connection(port, autoreconnect=True, baud=baudrate)
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self._mav.wait_heartbeat(timeout=10)

    def _send(self, text: str) -> None:
        while text:
            chunk = text[:70]
            text = text[70:]
            payload = [ord(ch) for ch in chunk]
            payload.extend([0] * (70 - len(payload)))
            self._mav.mav.serial_control_send(
                10,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                len(chunk),
                payload,
            )

    def _recv(self, timeout_s: float = 0.1) -> str:
        msg = self._mav.recv_match(
            condition="SERIAL_CONTROL.count!=0",
            type="SERIAL_CONTROL",
            blocking=True,
            timeout=timeout_s,
        )
        if msg is None:
            return ""
        return "".join(chr(ch) for ch in msg.data[: msg.count])

    def read_until_prompt(self, timeout_s: float = 8.0, quiet_timeout_s: float = 1.0) -> str:
        deadline = time.monotonic() + timeout_s
        chunks: list[str] = []
        last_data_time: float | None = None
        while time.monotonic() < deadline:
            data = self._recv(timeout_s=0.2)
            if data:
                chunks.append(data)
                last_data_time = time.monotonic()
                cleaned = ANSI_RE.sub("", "".join(chunks))
                if (
                    "nsh> " in cleaned
                    or cleaned.rstrip().endswith("nsh>")
                    or "pxh> " in cleaned
                    or cleaned.rstrip().endswith("pxh>")
                ):
                    return cleaned
            elif chunks and last_data_time is not None and (time.monotonic() - last_data_time) > quiet_timeout_s:
                return ANSI_RE.sub("", "".join(chunks))
        raise TimeoutError("Timed out waiting for nsh prompt")

    def run_command(self, command: str, timeout_s: float = 8.0) -> str:
        self._send("\n")
        try:
            self.read_until_prompt(timeout_s=2.0)
        except TimeoutError:
            pass
        self._send(command + "\n")
        output = self.read_until_prompt(timeout_s=timeout_s)
        return ANSI_RE.sub("", output)

    def close(self) -> None:
        self._mav.mav.serial_control_send(10, 0, 0, 0, 0, [0] * 70)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run non-interactive MAVLink shell smoke commands on a PX4 board.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial/MAVLink device, default /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=57600, help="MAVLink serial baudrate")
    parser.add_argument("--cmd", action="append", required=True, help="Shell command to run; repeat for multiple commands")
    parser.add_argument("--require", action="append", default=[], help="Substring that must appear somewhere in the combined output")
    args = parser.parse_args()

    shell = MavlinkShell(args.port, args.baudrate)
    outputs: list[dict[str, str]] = []

    try:
        for command in args.cmd:
            result = shell.run_command(command)
            outputs.append({"command": command, "output": result})
    finally:
        shell.close()

    combined = "\n".join(item["output"] for item in outputs)
    missing = [pattern for pattern in args.require if pattern not in combined]
    payload = {
        "ok": not missing,
        "port": args.port,
        "baudrate": args.baudrate,
        "commands": outputs,
        "missing": missing,
    }
    print(json.dumps(payload, indent=2))
    return 0 if not missing else 1


if __name__ == "__main__":
    sys.exit(main())
