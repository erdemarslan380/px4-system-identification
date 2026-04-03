#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_hitl_udp_sequence import (
    decode_statustext,
    prepare_hover_and_anchor,
    request_message_interval,
    set_param,
    set_position_target_relative,
    start_gcs_heartbeat_thread,
    wait_heartbeat,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a CDC-only HIL takeoff and offboard hover-hold diagnostic."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--hover-z", type=float, default=-5.0)
    parser.add_argument("--hover-timeout", type=float, default=45.0)
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    parser.add_argument("--hold-seconds", type=float, default=20.0)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--hold-setpoint-period", type=float, default=0.2)
    parser.add_argument("--manual-control-mode", type=int, choices=range(0, 9), default=4)
    parser.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    parser.add_argument("--arm-attempts", type=int, default=3)
    parser.add_argument("--sim-ready-timeout", type=float, default=20.0)
    parser.add_argument("--sim-ready-min-local-samples", type=int, default=3)
    parser.add_argument("--allow-missing-local-position", action="store_true")
    parser.add_argument("--blind-hover-seconds", type=float, default=12.0)
    return parser.parse_args()


def heartbeat_main_mode(msg) -> int:
    return (int(getattr(msg, "custom_mode", 0)) >> 16) & 0xFF


def wait_for_position_mode(mav, timeout: float = 8.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        if heartbeat_main_mode(msg) == 3:
            return
    raise TimeoutError("Position mode was not confirmed by HEARTBEAT")


def set_position_mode(mav) -> None:
    custom_mode = 3 << 16
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        29,
        3,
        0,
        0,
        0,
        0,
        0,
    )
    wait_for_position_mode(mav)


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
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        prepare_hover_and_anchor(
            mav,
            hover_z=args.hover_z,
            hover_timeout=args.hover_timeout,
            settle_seconds=args.settle_seconds,
            arm_attempts=args.arm_attempts,
            manual_control_mode=args.manual_control_mode,
            pre_offboard_seconds=args.pre_offboard_seconds,
            sim_ready_timeout=args.sim_ready_timeout,
            sim_ready_min_local_samples=args.sim_ready_min_local_samples,
            allow_missing_local_position=args.allow_missing_local_position,
            blind_hover_seconds=args.blind_hover_seconds,
        )

        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        print(f"Holding hover for {args.hold_seconds:.1f} s", flush=True)
        deadline = time.time() + args.hold_seconds
        next_setpoint = 0.0
        next_report = 0.0
        max_xy = 0.0
        max_tilt = 0.0

        while time.time() < deadline:
            now = time.time()
            if now >= next_setpoint:
                set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
                next_setpoint = now + args.hold_setpoint_period

            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg_type == "ATTITUDE":
                tilt = math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch))))
                max_tilt = max(max_tilt, tilt)
                if now >= next_report:
                    print(
                        f"ATT roll={math.degrees(float(msg.roll)):.2f} pitch={math.degrees(float(msg.pitch)):.2f} yaw={math.degrees(float(msg.yaw)):.2f}",
                        flush=True,
                    )
                    next_report = now + args.report_period
                continue

            xy = math.hypot(float(msg.x), float(msg.y))
            max_xy = max(max_xy, xy)
            if now >= next_report:
                print(
                    f"LPOS x={float(msg.x):.3f} y={float(msg.y):.3f} z={float(msg.z):.3f} "
                    f"vx={float(msg.vx):.3f} vy={float(msg.vy):.3f} vz={float(msg.vz):.3f}",
                    flush=True,
                )
                next_report = now + args.report_period

        print(f"Hover summary: max_xy={max_xy:.3f} m max_tilt={max_tilt:.2f} deg", flush=True)
        set_position_mode(mav)
        print("Returned to Position mode", flush=True)
        return 0
    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
