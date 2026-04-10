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
    set_anchor_from_position,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_builtin_trajectory_minimal import TRAJ_DURATION_S, observe_builtin_trajectory
from run_hitl_px4_mavlink_takeoff_then_offboard_hold import (
    ManualControlThread,
    best_effort_disarm,
    set_position_mode,
    throttle_for_target,
    wait_for_attitude,
    wait_for_manual_hover,
)
from run_hitl_px4_trajectory_reader_position_step_minimal import send_position_target_local_ned


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="HIL built-in trajectory runner with manual Position takeoff, then OFFBOARD/custom_pos trajectory."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--traj-id", type=int, choices=sorted(TRAJ_DURATION_S), default=100)
    parser.add_argument("--hover-z", type=float, default=-3.0)
    parser.add_argument("--climb-timeout", type=float, default=25.0)
    parser.add_argument("--manual-period", type=float, default=0.05)
    parser.add_argument("--setpoint-period", type=float, default=0.05)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--arm-attempts", type=int, default=3)
    parser.add_argument("--hover-z-tolerance", type=float, default=0.35)
    parser.add_argument("--offboard-entry-z-margin", type=float, default=0.35)
    parser.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    parser.add_argument("--manual-stable-window", type=float, default=1.5)
    parser.add_argument("--manual-stable-horiz-speed", type=float, default=0.25)
    parser.add_argument("--manual-stable-vert-speed", type=float, default=0.12)
    parser.add_argument("--manual-stable-tilt-deg", type=float, default=8.0)
    parser.add_argument("--manual-xy-limit", type=float, default=0.15)
    parser.add_argument("--offboard-capture-seconds", type=float, default=10.0)
    parser.add_argument("--capture-xy-limit", type=float, default=0.25)
    parser.add_argument("--capture-z-tolerance", type=float, default=0.25)
    parser.add_argument("--capture-tilt-limit-deg", type=float, default=8.0)
    parser.add_argument("--capture-horiz-speed", type=float, default=0.18)
    parser.add_argument("--capture-vert-speed", type=float, default=0.10)
    parser.add_argument("--capture-stable-window", type=float, default=3.0)
    parser.add_argument("--custom-hold-timeout", type=float, default=8.0)
    parser.add_argument("--custom-xy-limit", type=float, default=0.20)
    parser.add_argument("--custom-z-tolerance", type=float, default=0.20)
    parser.add_argument("--custom-tilt-limit-deg", type=float, default=6.0)
    parser.add_argument("--custom-horiz-speed", type=float, default=0.20)
    parser.add_argument("--custom-vert-speed", type=float, default=0.10)
    parser.add_argument("--custom-stable-window", type=float, default=1.0)
    parser.add_argument("--trajectory-tail-seconds", type=float, default=10.0)
    parser.add_argument("--trajectory-xy-envelop-limit", type=float, default=3.50)
    parser.add_argument("--trajectory-z-tolerance", type=float, default=0.50)
    parser.add_argument("--trajectory-tilt-limit-deg", type=float, default=20.0)
    parser.add_argument("--ground-xy-window", type=float, default=0.25)
    parser.add_argument("--ground-z-window", type=float, default=0.25)
    return parser.parse_args()


def try_set_param(mav, name: str, value, param_type: int) -> None:
    try:
        set_param(mav, name, value, param_type)
    except Exception as exc:
        print(f"Best-effort param set skipped: {name}={value} ({exc})", flush=True)


def wait_for_stable_hold(
    mav,
    *,
    label: str,
    timeout_s: float,
    stable_window_s: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    xy_limit: float,
    z_tolerance: float,
    tilt_limit_deg: float,
    horiz_speed_limit: float,
    vert_speed_limit: float,
    report_period: float,
    send_direct_setpoint: bool,
    setpoint_period: float,
    yaw: float,
):
    deadline = time.time() + timeout_s
    stable_since = None
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    max_horiz_speed = 0.0
    max_vert_speed = 0.0
    failsafe_seen = False

    while time.time() < deadline:
        if send_direct_setpoint:
            send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)
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
            xy_err = math.hypot(float(msg.x - ref_x), float(msg.y - ref_y))
            z_err = abs(float(msg.z - ref_z))
            horiz_speed = math.hypot(float(msg.vx), float(msg.vy))
            vert_speed = abs(float(msg.vz))
            max_xy = max(max_xy, xy_err)
            max_z_err = max(max_z_err, z_err)
            max_horiz_speed = max(max_horiz_speed, horiz_speed)
            max_vert_speed = max(max_vert_speed, vert_speed)

        elif mtype == "ATTITUDE":
            latest_att = msg
            tilt = math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch))))
            max_tilt = max(max_tilt, tilt)

        now = time.time()
        if latest_lpos and latest_att and now >= next_report:
            print(
                f"{label} x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if latest_lpos and latest_att:
            xy_err = math.hypot(float(latest_lpos.x - ref_x), float(latest_lpos.y - ref_y))
            z_err = abs(float(latest_lpos.z - ref_z))
            horiz_speed = math.hypot(float(latest_lpos.vx), float(latest_lpos.vy))
            vert_speed = abs(float(latest_lpos.vz))
            tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
            stable = (
                xy_err <= xy_limit
                and z_err <= z_tolerance
                and horiz_speed <= horiz_speed_limit
                and vert_speed <= vert_speed_limit
                and tilt <= tilt_limit_deg
            )
            if stable:
                stable_since = stable_since or now
                if now - stable_since >= stable_window_s:
                    return (
                        latest_lpos,
                        latest_att,
                        max_xy,
                        max_z_err,
                        max_tilt,
                        max_horiz_speed,
                        max_vert_speed,
                        failsafe_seen,
                        True,
                    )
            else:
                stable_since = None

    if latest_lpos is None or latest_att is None:
        raise RuntimeError(f"No telemetry during {label}")
    return (
        latest_lpos,
        latest_att,
        max_xy,
        max_z_err,
        max_tilt,
        max_horiz_speed,
        max_vert_speed,
        failsafe_seen,
        False,
    )


def main() -> int:
    args = parse_args()
    kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    manual = ManualControlThread(mav, args.manual_period)
    manual.start()
    successful_completion = False

    try:
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        set_param(mav, "COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_ARM_WO_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_RC_LOSS_T", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_FAIL_ACT_T", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OF_LOSS_T", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OBL_RC_ACT", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DL_LOSS_T", 30, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "NAV_RCL_ACT", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_RCL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DLL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "GF_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_IO_SAFETY", 22027, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_USB_CHK", 197848, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTMODE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTM_BTN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart HIL clean before running manual-takeoff trajectory."
            )

        att0 = wait_for_attitude(mav)
        yaw0 = float(att0.yaw)
        target_x = baseline_x
        target_y = baseline_y
        target_z = baseline_z + args.hover_z
        print(
            f"Manual takeoff trajectory target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} "
            f"yaw={math.degrees(yaw0):.1f}deg traj_id={args.traj_id}",
            flush=True,
        )

        set_position_mode(mav)
        manual.update(x=0, y=0, z=0, r=0)
        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        climb_deadline = time.time() + args.climb_timeout
        latest_lpos = None
        latest_att = None
        next_report = 0.0

        while time.time() < climb_deadline:
            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.manual_period)
            if not msg:
                manual.update(z=850)
                continue

            if msg.get_type() == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_lpos = msg
                manual.update(z=throttle_for_target(float(msg.z), target_z, float(msg.vz)))
            elif msg.get_type() == "ATTITUDE":
                latest_att = msg

            now = time.time()
            if latest_lpos and latest_att and now >= next_report:
                print(
                    f"Manual takeoff x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                    f"thr={throttle_for_target(float(latest_lpos.z), target_z, float(latest_lpos.vz))} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

            if (
                latest_lpos
                and float(latest_lpos.z) <= target_z + args.hover_z_tolerance
                and abs(float(latest_lpos.vz)) <= max(0.8, args.manual_stable_vert_speed * 8.0)
            ):
                break

        capture_ready = False
        if latest_lpos is not None and latest_att is not None:
            current_z = float(latest_lpos.z)
            horiz_speed = math.hypot(float(latest_lpos.vx), float(latest_lpos.vy))
            vert_speed = abs(float(latest_lpos.vz))
            tilt_deg = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
            capture_ready = (
                abs(current_z - target_z) <= args.hover_z_tolerance
                and horiz_speed <= max(1.5, args.manual_stable_horiz_speed * 6.0)
                and vert_speed <= max(0.8, args.manual_stable_vert_speed * 8.0)
                and tilt_deg <= max(20.0, args.manual_stable_tilt_deg * 3.0)
            )

        if capture_ready:
            hover_msg = latest_lpos
            hover_att = latest_att
            print(
                "Manual takeoff reached capture gate without full hover settle; switching to OFFBOARD capture.",
                flush=True,
            )
        else:
            hover_msg, hover_att = wait_for_manual_hover(
                mav,
                target_x=target_x,
                target_y=target_y,
                target_z=target_z,
                min_entry_z=target_z + args.offboard_entry_z_margin,
                z_tolerance=args.hover_z_tolerance,
                timeout=max(5.0, args.climb_timeout),
                max_horiz_speed=args.manual_stable_horiz_speed,
                max_vert_speed=args.manual_stable_vert_speed,
                max_tilt_deg=args.manual_stable_tilt_deg,
                max_xy=args.manual_xy_limit,
                stable_window=args.manual_stable_window,
                report_period=args.report_period,
                manual=manual,
            )

        capture_x = float(hover_msg.x)
        capture_y = float(hover_msg.y)
        capture_z = float(hover_msg.z)
        capture_yaw = float(hover_att.yaw)
        manual.update(z=500)
        print(
            f"Switching to OFFBOARD capture at x={capture_x:.3f} y={capture_y:.3f} z={capture_z:.3f}",
            flush=True,
        )

        pre_deadline = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < pre_deadline:
            manual.update(z=500)
            send_position_target_local_ned(mav, capture_x, capture_y, capture_z, capture_yaw)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)
        manual.stop()

        (
            latest_lpos,
            latest_att,
            capture_max_xy,
            capture_max_z_err,
            capture_max_tilt,
            capture_max_horiz_speed,
            capture_max_vert_speed,
            capture_failsafe,
            capture_stable,
        ) = wait_for_stable_hold(
            mav,
            label="Offboard capture",
            timeout_s=args.offboard_capture_seconds,
            stable_window_s=args.capture_stable_window,
            ref_x=capture_x,
            ref_y=capture_y,
            ref_z=capture_z,
            xy_limit=args.capture_xy_limit,
            z_tolerance=args.capture_z_tolerance,
            tilt_limit_deg=args.capture_tilt_limit_deg,
            horiz_speed_limit=args.capture_horiz_speed,
            vert_speed_limit=args.capture_vert_speed,
            report_period=args.report_period,
            send_direct_setpoint=True,
            setpoint_period=args.setpoint_period,
            yaw=capture_yaw,
        )
        if capture_failsafe:
            raise RuntimeError(
                f"Offboard capture failed with failsafe: max_xy={capture_max_xy:.3f}m "
                f"max_z_err={capture_max_z_err:.3f}m max_tilt={capture_max_tilt:.2f}deg"
            )
        if not capture_stable:
            print(
                f"Offboard capture warning: max_xy={capture_max_xy:.3f}m "
                f"max_z_err={capture_max_z_err:.3f}m max_tilt={capture_max_tilt:.2f}deg "
                f"max_horiz_speed={capture_max_horiz_speed:.3f}m/s max_vert_speed={capture_max_vert_speed:.3f}m/s",
                flush=True,
            )

        set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", float(latest_lpos.x), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", float(latest_lpos.y), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", float(latest_lpos.z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", float(latest_att.yaw), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("custom_pos_control enabled", flush=True)

        custom_ref_x = float(latest_lpos.x)
        custom_ref_y = float(latest_lpos.y)
        custom_ref_z = float(latest_lpos.z)
        (
            latest_lpos,
            latest_att,
            custom_max_xy,
            custom_max_z_err,
            custom_max_tilt,
            custom_max_horiz_speed,
            custom_max_vert_speed,
            custom_failsafe,
            custom_stable,
        ) = wait_for_stable_hold(
            mav,
            label="Custom hold",
            timeout_s=args.custom_hold_timeout,
            stable_window_s=args.custom_stable_window,
            ref_x=custom_ref_x,
            ref_y=custom_ref_y,
            ref_z=custom_ref_z,
            xy_limit=args.custom_xy_limit,
            z_tolerance=args.custom_z_tolerance,
            tilt_limit_deg=args.custom_tilt_limit_deg,
            horiz_speed_limit=args.custom_horiz_speed,
            vert_speed_limit=args.custom_vert_speed,
            report_period=args.report_period,
            send_direct_setpoint=False,
            setpoint_period=args.setpoint_period,
            yaw=float(latest_att.yaw),
        )
        if custom_failsafe:
            raise RuntimeError(
                f"Custom hold failsafe: max_xy={custom_max_xy:.3f}m max_z_err={custom_max_z_err:.3f}m "
                f"max_tilt={custom_max_tilt:.2f}deg max_horiz_speed={custom_max_horiz_speed:.3f}m/s "
                f"max_vert_speed={custom_max_vert_speed:.3f}m/s failsafe={custom_failsafe} "
                f"stable={custom_stable}"
            )

        if not custom_stable:
            print(
                "Custom hold warning: "
                f"max_xy={custom_max_xy:.3f}m max_z_err={custom_max_z_err:.3f}m "
                f"max_tilt={custom_max_tilt:.2f}deg max_horiz_speed={custom_max_horiz_speed:.3f}m/s "
                f"max_vert_speed={custom_max_vert_speed:.3f}m/s stable={custom_stable}. "
                "Continuing to built-in trajectory.",
                flush=True,
            )

        anchor_x = float(latest_lpos.x)
        anchor_y = float(latest_lpos.y)
        anchor_z = float(latest_lpos.z)
        set_anchor_from_position(mav, anchor_x, anchor_y, anchor_z)
        set_param(mav, "TRJ_ACTIVE_ID", args.traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print(
            f"trajectory_reader built-in trajectory id={args.traj_id} anchor x={anchor_x:.3f} y={anchor_y:.3f} z={anchor_z:.3f}",
            flush=True,
        )
        set_param(mav, "TRJ_MODE_CMD", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("trajectory_reader built-in trajectory started", flush=True)

        latest_lpos, latest_att, traj_max_xy, traj_max_z_err, traj_max_tilt, traj_failsafe = observe_builtin_trajectory(
            mav,
            traj_id=args.traj_id,
            duration_s=TRAJ_DURATION_S[args.traj_id] + args.trajectory_tail_seconds,
            anchor_x=anchor_x,
            anchor_y=anchor_y,
            anchor_z=anchor_z,
            report_period=args.report_period,
        )

        final_xy = math.hypot(float(latest_lpos.x), float(latest_lpos.y))
        final_z_err = abs(float(latest_lpos.z) - anchor_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        print(
            f"Manual-takeoff built-in trajectory summary: traj_id={args.traj_id} max_xy={traj_max_xy:.3f}m "
            f"max_z_err={traj_max_z_err:.3f}m max_tilt={traj_max_tilt:.2f}deg "
            f"final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m final_tilt={final_tilt:.2f}deg "
            f"failsafe={traj_failsafe}",
            flush=True,
        )

        if (
            traj_failsafe
            or traj_max_xy > args.trajectory_xy_envelop_limit
            or traj_max_z_err > args.trajectory_z_tolerance
            or traj_max_tilt > args.trajectory_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Built-in trajectory unstable: max_xy={traj_max_xy:.3f}m max_z_err={traj_max_z_err:.3f}m "
                f"max_tilt={traj_max_tilt:.2f}deg failsafe={traj_failsafe}"
            )

        print("manual-takeoff trajectory_reader built-in trajectory stable", flush=True)
        successful_completion = True
        return 0
    finally:
        manual.update(z=500)
        try:
            manual.stop()
        except Exception:
            pass
        if not successful_completion:
            best_effort_disarm(mav)
            try:
                set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            except Exception:
                pass
            try:
                set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            except Exception:
                pass
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
