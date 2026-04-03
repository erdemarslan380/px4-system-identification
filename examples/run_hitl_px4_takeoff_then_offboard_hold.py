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
    arm_with_retries,
    decode_statustext,
    request_message_interval,
    send_nav_takeoff,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_local_position,
    wait_for_sim_ready,
    wait_heartbeat,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baseline HIL: PX4 takeoff first, then switch to OFFBOARD pose hold at hover."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--hover-z", type=float, default=-2.0)
    parser.add_argument("--hover-timeout", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    parser.add_argument("--setpoint-period", type=float, default=0.05)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--stable-window", type=float, default=2.0)
    parser.add_argument("--max-horiz-speed", type=float, default=0.15)
    parser.add_argument("--max-vert-speed", type=float, default=0.10)
    parser.add_argument("--max-tilt-before-offboard-deg", type=float, default=5.0)
    parser.add_argument("--xy-limit", type=float, default=0.25)
    parser.add_argument("--z-tolerance", type=float, default=0.25)
    parser.add_argument("--tilt-limit-deg", type=float, default=10.0)
    return parser.parse_args()


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


def wait_for_stable_hover(
    mav,
    *,
    target_z: float,
    timeout: float,
    z_tolerance: float,
    max_horiz_speed: float,
    max_vert_speed: float,
    max_tilt_deg: float,
    stable_window: float,
    report_period: float,
):
    deadline = time.time() + timeout
    latest_lpos = None
    latest_att = None
    stable_since = None
    next_report = 0.0

    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            latest_lpos = msg
        elif msg.get_type() == "ATTITUDE":
            latest_att = msg

        now = time.time()
        if latest_lpos and latest_att and now >= next_report:
            print(
                f"Takeoff phase x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if not latest_lpos or not latest_att:
            continue

        z_err = abs(float(latest_lpos.z) - target_z)
        horiz_speed = math.hypot(float(latest_lpos.vx), float(latest_lpos.vy))
        vert_speed = abs(float(latest_lpos.vz))
        tilt_deg = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

        stable = (
            z_err <= z_tolerance
            and horiz_speed <= max_horiz_speed
            and vert_speed <= max_vert_speed
            and tilt_deg <= max_tilt_deg
        )

        if stable:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= stable_window:
                return latest_lpos, latest_att
        else:
            stable_since = None

    raise TimeoutError(
        f"Vehicle did not settle into hover around z={target_z:.3f}; "
        f"last_lpos={latest_lpos} last_att={latest_att}"
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
        set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "MIS_TAKEOFF_ALT", max(0.5, abs(float(args.hover_z))), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        sim_ready_msg = wait_for_sim_ready(
            mav,
            timeout=20.0,
            require_local_position=True,
            min_local_samples=3,
        )

        baseline_x = float(sim_ready_msg.x)
        baseline_y = float(sim_ready_msg.y)
        baseline_z = float(sim_ready_msg.z)
        target_z = baseline_z + args.hover_z

        print(
            f"Takeoff target x={baseline_x:.3f} y={baseline_y:.3f} z={target_z:.3f}",
            flush=True,
        )

        send_nav_takeoff(mav)
        arm_with_retries(mav, attempts=3)

        hold_msg, att_msg = wait_for_stable_hover(
            mav,
            target_z=target_z,
            timeout=args.hover_timeout,
            z_tolerance=args.z_tolerance,
            max_horiz_speed=args.max_horiz_speed,
            max_vert_speed=args.max_vert_speed,
            max_tilt_deg=args.max_tilt_before_offboard_deg,
            stable_window=args.stable_window,
            report_period=args.report_period,
        )

        if args.settle_seconds > 0.0:
            time.sleep(args.settle_seconds)
            hold_msg = wait_for_local_position(mav, timeout=3.0)

        hold_x = float(hold_msg.x)
        hold_y = float(hold_msg.y)
        hold_z = float(hold_msg.z)
        hold_yaw = float(att_msg.yaw) if att_msg else 0.0

        print(
            f"Switching to OFFBOARD hold at x={hold_x:.3f} y={hold_y:.3f} z={hold_z:.3f}",
            flush=True,
        )

        deadline = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < deadline:
            send_position_target_local_ned(mav, hold_x, hold_y, hold_z, hold_yaw)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        latest_lpos = hold_msg
        latest_att = att_msg
        max_xy = 0.0
        max_tilt = 0.0
        success_window_start = None
        next_report = 0.0
        end = time.time() + args.hold_seconds

        while time.time() < end:
            send_position_target_local_ned(mav, hold_x, hold_y, hold_z, hold_yaw)

            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.setpoint_period)
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg_type == "LOCAL_POSITION_NED":
                latest_lpos = msg
                xy = math.hypot(float(msg.x - hold_x), float(msg.y - hold_y))
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
                xy_err = math.hypot(float(latest_lpos.x - hold_x), float(latest_lpos.y - hold_y))
                z_err = abs(float(latest_lpos.z) - hold_z)
                tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

                if xy_err <= args.xy_limit and z_err <= args.z_tolerance and tilt <= args.tilt_limit_deg:
                    if success_window_start is None:
                        success_window_start = now
                else:
                    success_window_start = None

        print(f"Takeoff->OFFBOARD hold summary: max_xy={max_xy:.3f}m max_tilt={max_tilt:.2f}deg", flush=True)
        return 0

    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
