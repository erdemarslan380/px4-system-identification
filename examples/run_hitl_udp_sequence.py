#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.hitl_catalog import (
    identification_duration_s,
    identification_profile_index,
    trajectory_duration_s,
)


def normalize_param_id(param_id: object) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode(errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def decode_param_value(raw_value: float, param_type: int) -> float | int:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        packed = struct.pack("<f", float(raw_value))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<b", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<B", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<h", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<H", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<i", packed)[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<I", packed)[0]
    return float(raw_value)


def encode_param_value(value: float | int, param_type: int) -> float:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        int_value = int(round(float(value)))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<f", struct.pack("<bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<f", struct.pack("<Bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<f", struct.pack("<hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<f", struct.pack("<Hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<f", struct.pack("<i", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<f", struct.pack("<I", int_value))[0]
    return float(value)


def param_value_matches(raw_value: float, expected: float, param_type: int) -> bool:
    if math.isclose(float(raw_value), float(expected), rel_tol=0.0, abs_tol=1e-3):
        return True
    decoded = decode_param_value(raw_value, param_type)
    if isinstance(decoded, int):
        return decoded == int(round(expected))
    return math.isclose(float(decoded), float(expected), rel_tol=0.0, abs_tol=1e-3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one HITL identification profile or trajectory over a MAVLink endpoint."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud if --endpoint is a device path.")
    parser.add_argument("--kind", choices=["ident", "trajectory"], required=True)
    parser.add_argument("--name", help="Identification profile name.")
    parser.add_argument("--traj-id", type=int, help="Trajectory id.")
    parser.add_argument(
        "--hover-z",
        type=float,
        default=-3.0,
        help="Desired relative z step from the current LOCAL_POSITION_NED z at OFFBOARD entry.",
    )
    parser.add_argument("--hover-timeout", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=6.0)
    parser.add_argument(
        "--allow-missing-local-position",
        action="store_true",
        help="For HIL links that do not stream LOCAL_POSITION_NED, use a timed hover settle and keep the existing anchor params.",
    )
    parser.add_argument(
        "--blind-hover-seconds",
        type=float,
        default=12.0,
        help="Timed settle used with --allow-missing-local-position when LOCAL_POSITION_NED is unavailable.",
    )
    parser.add_argument("--tail-seconds", type=float, default=3.0)
    parser.add_argument(
        "--strict-reset",
        action="store_true",
        help="Fail if resetting TRJ_MODE_CMD back to position mode is not confirmed at the end of the run.",
    )
    return parser.parse_args()


def wait_heartbeat(mav, timeout: float = 20.0) -> None:
    heartbeat = mav.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise TimeoutError("No UDP heartbeat received from jMAVSim/PX4")


def send_gcs_heartbeat(mav) -> None:
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def decode_statustext(msg) -> str:
    text = getattr(msg, "text", "")
    if isinstance(text, bytes):
        return text.decode(errors="ignore").rstrip("\x00")
    return str(text).rstrip("\x00")


def start_gcs_heartbeat_thread(mav, period_s: float = 1.0) -> tuple[threading.Event, threading.Thread]:
    stop_event = threading.Event()

    def _worker() -> None:
        while not stop_event.is_set():
            send_gcs_heartbeat(mav)
            stop_event.wait(period_s)

    thread = threading.Thread(target=_worker, name="gcs-heartbeat", daemon=True)
    thread.start()
    return stop_event, thread


def wait_command_ack(mav, command: int, timeout: float = 5.0) -> int:
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return int(msg.result)
    raise TimeoutError(f"No COMMAND_ACK for {command}")


def set_param(mav, name: str, value: float, param_type: int, timeout: float = 8.0) -> None:
    encoded_value = encode_param_value(value, param_type)
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode("ascii"),
        float(encoded_value),
        param_type,
    )
    end = time.time() + timeout
    while time.time() < end:
        mav.mav.param_request_read_send(
            mav.target_system,
            mav.target_component,
            name.encode("ascii"),
            -1,
        )
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        param_id = normalize_param_id(msg.param_id)
        if param_id != name:
            continue
        if param_value_matches(float(msg.param_value), float(value), param_type):
            return
    raise TimeoutError(f"PARAM_VALUE confirmation missing for {name}={value}")


def read_param(mav, name: str, param_type: int, timeout: float = 8.0) -> float | int:
    end = time.time() + timeout
    while time.time() < end:
        mav.mav.param_request_read_send(
            mav.target_system,
            mav.target_component,
            name.encode("ascii"),
            -1,
        )
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        param_id = normalize_param_id(msg.param_id)
        if param_id != name:
            continue
        return decode_param_value(float(msg.param_value), param_type)
    raise TimeoutError(f"Failed to read parameter {name}")


def arm(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    result = wait_command_ack(mav, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Arm rejected with ACK result {result}")


def set_offboard(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        29,
        6,
        0,
        0,
        0,
        0,
        0,
    )
    result = wait_command_ack(mav, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"OFFBOARD rejected with ACK result {result}")


def wait_for_local_position(mav, timeout: float = 5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg:
            return msg
    raise TimeoutError("No LOCAL_POSITION_NED received")


def wait_for_hover(mav, target_delta_z: float, timeout: float):
    first_msg = wait_for_local_position(mav, timeout=min(timeout, 5.0))
    baseline_z = float(first_msg.z)
    target_z = baseline_z + target_delta_z
    print(f"Hover baseline z={baseline_z:.3f}, target z={target_z:.3f}", flush=True)
    end = time.time() + timeout
    last_msg = first_msg
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if not msg:
            continue
        z = float(msg.z)
        print(f"LOCAL_POSITION_NED z={z:.3f}", flush=True)
        last_msg = msg
        if abs(z - target_z) <= 0.5:
            return msg
    raise TimeoutError(f"Vehicle did not reach hover band around z={target_z}")


def set_anchor_from_position(mav, x: float, y: float, z: float) -> None:
    set_param(mav, "TRJ_ANCHOR_X", x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_ANCHOR_Y", y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_ANCHOR_Z", z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def reset_mode_cmd(mav, strict: bool) -> bool:
    try:
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        return True
    except TimeoutError as exc:
        if strict:
            raise
        print(f"Warning: final TRJ_MODE_CMD reset was not confirmed: {exc}", flush=True)
        return False


def prepare_hover_and_anchor(
    mav,
    *,
    hover_z: float,
    hover_timeout: float,
    settle_seconds: float,
    allow_missing_local_position: bool,
    blind_hover_seconds: float,
) -> None:
    arm(mav)
    set_offboard(mav)

    try:
        wait_for_hover(mav, hover_z, hover_timeout)
        time.sleep(settle_seconds)
        anchor_msg = wait_for_local_position(mav, timeout=3.0)
    except TimeoutError:
        if not allow_missing_local_position:
            raise
        print(
            f"LOCAL_POSITION_NED not available; using blind hover settle for {blind_hover_seconds:.1f} s and the existing TRJ_ANCHOR_* params",
            flush=True,
        )
        time.sleep(blind_hover_seconds)
        return

    set_anchor_from_position(mav, float(anchor_msg.x), float(anchor_msg.y), float(anchor_msg.z))
    print(
        f"Anchor set from LOCAL_POSITION_NED: x={float(anchor_msg.x):.3f}, "
        f"y={float(anchor_msg.y):.3f}, z={float(anchor_msg.z):.3f}",
        flush=True,
    )


def wait_run_window(mav, duration_s: float) -> None:
    deadline = time.time() + duration_s
    while time.time() < deadline:
        timeout = max(0.0, min(1.0, deadline - time.time()))
        msg = mav.recv_match(blocking=True, timeout=timeout)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud
    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)

    try:
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        if args.kind == "trajectory":
            if args.traj_id is None:
                raise SystemExit("--traj-id is required for --kind trajectory")
            set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_ACTIVE_ID", args.traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            run_duration = trajectory_duration_s(args.traj_id)
            trigger_name = f"trajectory {args.traj_id}"
            trigger_value = 1
        else:
            if not args.name:
                raise SystemExit("--name is required for --kind ident")
            set_param(mav, "CST_POS_CTRL_TYP", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(
                mav,
                "TRJ_IDENT_PROF",
                identification_profile_index(args.name),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            run_duration = identification_duration_s(args.name)
            trigger_name = f"identification {args.name}"
            trigger_value = 2

        prepare_hover_and_anchor(
            mav,
            hover_z=args.hover_z,
            hover_timeout=args.hover_timeout,
            settle_seconds=args.settle_seconds,
            allow_missing_local_position=args.allow_missing_local_position,
            blind_hover_seconds=args.blind_hover_seconds,
        )

        print(f"Starting {trigger_name}", flush=True)
        set_param(mav, "TRJ_MODE_CMD", trigger_value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        wait_run_window(mav, run_duration + args.tail_seconds)
        reset_mode_cmd(mav, strict=args.strict_reset)
        print(f"Completed {trigger_name}", flush=True)
    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.5)
        mav.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
