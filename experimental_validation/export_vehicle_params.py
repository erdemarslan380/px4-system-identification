#!/usr/bin/env python3
"""Export a full PX4 parameter snapshot in a QGC-compatible format."""

from __future__ import annotations

import argparse
import json
import struct
import time
from pathlib import Path

from pymavlink import mavutil


def format_qgc_parameter_dump(
    params: dict[str, tuple[int | float, int]],
    system_id: int,
    component_id: int,
) -> str:
    lines = ["# PX4 parameter snapshot exported by experimental_validation/export_vehicle_params.py"]
    for name in sorted(params):
        value, param_type = params[name]
        lines.append(f"{system_id}\t{component_id}\t{name}\t{value}\t{param_type}")
    return "\n".join(lines) + "\n"


def decode_param_value(param_value: float, param_type: int) -> int | float:
    mavlink = mavutil.mavlink
    raw = struct.pack("<f", float(param_value))

    if param_type == mavlink.MAV_PARAM_TYPE_REAL32:
        return struct.unpack("<f", raw)[0]
    if param_type == mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack("<B", raw[:1])[0]
    if param_type == mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack("<b", raw[:1])[0]
    if param_type == mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack("<H", raw[:2])[0]
    if param_type == mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack("<h", raw[:2])[0]
    if param_type == mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack("<I", raw)[0]
    if param_type == mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", raw)[0]

    return float(param_value)


def fetch_all_parameters(endpoint: str, baud: int, timeout_s: float) -> tuple[int, int, dict[str, tuple[int | float, int]]]:
    master = mavutil.mavlink_connection(endpoint, baud=baud if endpoint.startswith("/dev/") else None, autoreconnect=False)
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise RuntimeError(f"No MAVLink heartbeat received on {endpoint}")

    master.mav.param_request_list_send(master.target_system, master.target_component)
    params: dict[str, tuple[int | float, int]] = {}
    expected_count: int | None = None
    deadline = time.time() + timeout_s
    last_new_param_at = time.time()

    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            if expected_count is not None and len(params) >= expected_count and time.time() - last_new_param_at > 1.0:
                break
            continue

        param_id = msg.param_id
        if isinstance(param_id, bytes):
            name = param_id.decode("utf-8", errors="ignore").rstrip("\x00")
        else:
            name = str(param_id).rstrip("\x00")
        param_type = int(msg.param_type)
        params[name] = (decode_param_value(float(msg.param_value), param_type), param_type)
        expected_count = int(msg.param_count)
        last_new_param_at = time.time()

        if expected_count and len(params) >= expected_count and time.time() - last_new_param_at > 0.5:
            break

    if not params:
        raise RuntimeError(f"No parameters received from {endpoint}")

    return master.target_system, master.target_component, params


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Export a full PX4 parameter snapshot over MAVLink.")
    ap.add_argument("--endpoint", required=True, help="MAVLink endpoint, for example /dev/ttyACM1 or udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=57600, help="Baudrate for serial endpoints.")
    ap.add_argument("--timeout", type=float, default=30.0, help="Fetch timeout in seconds.")
    ap.add_argument("--out", required=True, help="Output .params path.")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    system_id, component_id, params = fetch_all_parameters(args.endpoint, args.baud, args.timeout)
    payload = format_qgc_parameter_dump(params, system_id, component_id)

    out_path = Path(args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(payload, encoding="utf-8")

    print(
        json.dumps(
            {
                "ok": True,
                "endpoint": args.endpoint,
                "system_id": system_id,
                "component_id": component_id,
                "param_count": len(params),
                "out": str(out_path),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
