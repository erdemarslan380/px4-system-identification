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
    request_message_interval,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_heartbeat,
    wait_for_sim_ready,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait for a manual RC takeoff, then switch PX4 baseline HIL into OFFBOARD hold."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--trigger-z", type=float, default=-0.8, help="Relative NED z threshold that starts OFFBOARD hold.")
    parser.add_argument("--hold-seconds", type=float, default=12.0)
    parser.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    parser.add_argument("--setpoint-period", type=float, default=0.05)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--xy-limit", type=float, default=0.35)
    parser.add_argument("--z-tolerance", type=float, default=0.35)
    parser.add_argument("--tilt-limit-deg", type=float, default=12.0)
    return parser.parse_args()


def heartbeat_is_armed(msg) -> bool:
    return bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_position_target_local_ned(mav, x: float, y: float, z: float, yaw: float) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        yaw,
        0.0,
    )


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)

    try:
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_RC_IN_MODE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        wait_for_sim_ready(
            mav,
            timeout=20.0,
            require_local_position=True,
            min_local_samples=3,
        )

        print(
            f"Waiting for manual takeoff. Arm in Position mode and climb until z <= {args.trigger_z:.2f}.",
            flush=True,
        )

        latest_lpos = None
        latest_att = None
        last_report = 0.0

        while True:
            msg = mav.recv_match(
                type=["HEARTBEAT", "LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"],
                blocking=True,
                timeout=0.5,
            )
            if not msg:
                continue

            msg_type = msg.get_type()
            now = time.time()

            if msg_type == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg_type == "LOCAL_POSITION_NED":
                latest_lpos = msg

            elif msg_type == "ATTITUDE":
                latest_att = msg

            elif msg_type == "HEARTBEAT":
                if not heartbeat_is_armed(msg):
                    continue

            if latest_lpos and latest_att and now >= last_report:
                print(
                    f"Manual phase x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                last_report = now + args.report_period

            if latest_lpos and latest_att and float(latest_lpos.z) <= args.trigger_z:
                break

        target_x = float(latest_lpos.x)
        target_y = float(latest_lpos.y)
        target_z = float(latest_lpos.z)
        target_yaw = float(latest_att.yaw)

        print(
            f"Manual takeoff detected. Holding current pose x={target_x:.3f} y={target_y:.3f} z={target_z:.3f}",
            flush=True,
        )

        pre_offboard_deadline = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < pre_offboard_deadline:
            send_position_target_local_ned(mav, target_x, target_y, target_z, target_yaw)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        max_xy = 0.0
        max_tilt = 0.0
        end = time.time() + args.hold_seconds
        next_report = 0.0
        success_window_start = None

        while time.time() < end:
            send_position_target_local_ned(mav, target_x, target_y, target_z, target_yaw)

            msg = mav.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"],
                blocking=True,
                timeout=args.setpoint_period,
            )
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg_type == "LOCAL_POSITION_NED":
                latest_lpos = msg
                xy = math.hypot(float(msg.x - target_x), float(msg.y - target_y))
                max_xy = max(max_xy, xy)

            elif msg_type == "ATTITUDE":
                latest_att = msg
                tilt = math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch))))
                max_tilt = max(max_tilt, tilt)

            now = time.time()
            if latest_lpos and latest_att and now >= next_report:
                print(
                    f"Hold phase x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

            if latest_lpos and latest_att:
                xy_err = math.hypot(float(latest_lpos.x - target_x), float(latest_lpos.y - target_y))
                z_err = abs(float(latest_lpos.z) - target_z)
                tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

                if xy_err <= args.xy_limit and z_err <= args.z_tolerance and tilt <= args.tilt_limit_deg:
                    if success_window_start is None:
                        success_window_start = now
                else:
                    success_window_start = None

        print(f"Manual offboard hold summary: max_xy={max_xy:.3f}m max_tilt={max_tilt:.2f}deg", flush=True)
        return 0

    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
