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
    robust_ground_baseline,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_sim_ready,
    wait_heartbeat,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Minimal module-managed takeoff and hover hold using trajectory_reader + custom_pos_control."
    )
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--hover-z", type=float, default=-3.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--settle-timeout", type=float, default=15.0)
    p.add_argument("--hold-seconds", type=float, default=5.0)
    p.add_argument("--report-period", type=float, default=0.5)
    p.add_argument("--arm-attempts", type=int, default=3)
    p.add_argument("--xy-limit", type=float, default=0.25)
    p.add_argument("--z-tolerance", type=float, default=0.20)
    p.add_argument("--tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--settle-radius", type=float, default=0.08)
    p.add_argument("--settle-vxy", type=float, default=0.08)
    p.add_argument("--settle-vz", type=float, default=0.08)
    return p.parse_args()


def disarm(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        21196,
        0,
        0,
        0,
        0,
        0,
    )


def observe_module_hold(
    mav,
    *,
    target_x: float,
    target_y: float,
    target_z: float,
    settle_timeout: float,
    hold_seconds: float,
    settle_radius: float,
    settle_vxy: float,
    settle_vz: float,
    report_period: float,
) -> tuple[object, object, float, float, float, bool]:
    latest_lpos = None
    latest_att = None
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    settle_since = None
    next_report = 0.0
    deadline = time.time() + max(1.0, settle_timeout)

    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
        if not msg:
            continue
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Failsafe activated" in text or "Flight termination active" in text:
                failsafe_seen = True
            continue
        if mtype == "LOCAL_POSITION_NED":
            latest_lpos = msg
            xy_err = math.hypot(float(msg.x) - target_x, float(msg.y) - target_y)
            z_err = abs(float(msg.z) - target_z)
            max_xy = max(max_xy, xy_err)
            max_z_err = max(max_z_err, z_err)
            speed_xy = math.hypot(float(msg.vx), float(msg.vy))
            speed_z = abs(float(msg.vz))
            if xy_err <= settle_radius and z_err <= settle_radius and speed_xy <= settle_vxy and speed_z <= settle_vz:
                settle_since = settle_since or time.time()
            else:
                settle_since = None
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))

        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"Module takeoff x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if settle_since and (time.time() - settle_since) >= hold_seconds:
            break

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during module takeoff hold")
    if not settle_since:
        raise RuntimeError("Module takeoff hold did not settle at the target")
    return latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen


def main() -> int:
    args = parse_args()
    kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)

    try:
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", args.hover_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        target_x = baseline_x
        target_y = baseline_y
        target_z = baseline_z + args.hover_z
        print(
            f"Module takeoff target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f}",
            flush=True,
        )

        arm_with_retries(mav, attempts=max(1, args.arm_attempts))
        time.sleep(max(0.5, args.pre_offboard_seconds))
        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen = observe_module_hold(
            mav,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            settle_timeout=args.settle_timeout,
            hold_seconds=args.hold_seconds,
            settle_radius=args.settle_radius,
            settle_vxy=args.settle_vxy,
            settle_vz=args.settle_vz,
            report_period=args.report_period,
        )

        final_xy = math.hypot(float(latest_lpos.x) - target_x, float(latest_lpos.y) - target_y)
        final_z_err = abs(float(latest_lpos.z) - target_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        print(
            f"Module takeoff summary: max_xy={max_xy:.3f}m max_z_err={max_z_err:.3f}m "
            f"max_tilt={max_tilt:.2f}deg final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m "
            f"final_tilt={final_tilt:.2f}deg failsafe={failsafe_seen}",
            flush=True,
        )

        if (
            failsafe_seen
            or max_xy > args.xy_limit
            or max_z_err > args.z_tolerance
            or max_tilt > args.tilt_limit_deg
            or final_xy > args.xy_limit
            or final_z_err > args.z_tolerance
            or final_tilt > args.tilt_limit_deg
        ):
            raise RuntimeError(
                f"Module takeoff unstable: max_xy={max_xy:.3f}m final_xy={final_xy:.3f}m "
                f"max_z_err={max_z_err:.3f}m final_z_err={final_z_err:.3f}m "
                f"max_tilt={max_tilt:.2f}deg final_tilt={final_tilt:.2f}deg failsafe={failsafe_seen}"
            )

        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3 << 16)
        disarm(mav)
        print("Module-managed takeoff/hold stable", flush=True)
        return 0
    finally:
        try:
            set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        except Exception:
            pass
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
