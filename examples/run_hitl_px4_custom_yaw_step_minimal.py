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
    wait_for_ground_quiet,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_trajectory_reader_position_step_minimal import execute_staged_hover_entry


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Minimal baseline -> custom_pos_control -> trajectory_reader yaw-step test.')
    p.add_argument('--endpoint', default='udpin:127.0.0.1:14550')
    p.add_argument('--hover-z', type=float, default=-3.5)
    p.add_argument('--ramp-seconds', type=float, default=12.0)
    p.add_argument('--settle-seconds', type=float, default=5.0)
    p.add_argument('--custom-hold-seconds', type=float, default=5.0)
    p.add_argument('--yaw-step-rad', type=float, default=0.8)
    p.add_argument('--yaw-step-seconds', type=float, default=10.0)
    p.add_argument('--pre-offboard-seconds', type=float, default=2.0)
    p.add_argument('--setpoint-period', type=float, default=0.05)
    p.add_argument('--report-period', type=float, default=0.5)
    p.add_argument('--arm-attempts', type=int, default=3)
    p.add_argument('--xy-limit', type=float, default=0.25)
    p.add_argument('--z-tolerance', type=float, default=0.25)
    p.add_argument('--tilt-limit-deg', type=float, default=12.0)
    p.add_argument('--yaw-success-rad', type=float, default=0.35)
    return p.parse_args()


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
        x, y, z,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        yaw,
        0.0,
    )


def wrap_pi(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


def observe_phase(mav, *, label: str, duration_s: float, report_period: float, ref_x: float, ref_y: float, ref_z: float,
                  send_direct_setpoint: bool = False, yaw_ref: float = 0.0):
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    max_yaw_err = 0.0
    yaw_min = float('inf')
    yaw_max = float('-inf')
    failsafe_seen = False
    while time.time() < end:
        if send_direct_setpoint:
            send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw_ref)
        msg = mav.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'STATUSTEXT'], blocking=True, timeout=0.2)
        if not msg:
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = decode_statustext(msg)
            print(text, flush=True)
            if 'Failsafe activated' in text or 'Flight termination active' in text:
                failsafe_seen = True
            continue
        if mtype == 'LOCAL_POSITION_NED':
            latest_lpos = msg
        elif mtype == 'ATTITUDE':
            latest_att = msg
        if not latest_lpos or not latest_att:
            continue
        xy = math.hypot(float(latest_lpos.x) - ref_x, float(latest_lpos.y) - ref_y)
        z_err = abs(float(latest_lpos.z) - ref_z)
        tilt = math.degrees(math.hypot(float(latest_att.roll), float(latest_att.pitch)))
        yaw_err = abs(wrap_pi(float(latest_att.yaw) - yaw_ref))
        yaw_min = min(yaw_min, float(latest_att.yaw))
        yaw_max = max(yaw_max, float(latest_att.yaw))
        max_xy = max(max_xy, xy)
        max_z_err = max(max_z_err, z_err)
        max_tilt = max(max_tilt, tilt)
        max_yaw_err = max(max_yaw_err, yaw_err)
        now = time.time()
        if now >= next_report:
            print(
                f"{label} x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"yaw={math.degrees(float(latest_att.yaw)):.1f}deg ref_yaw={math.degrees(yaw_ref):.1f}deg "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period
    yaw_span = 0.0 if yaw_min == float('inf') else abs(wrap_pi(yaw_max - yaw_min))
    return latest_lpos, latest_att, max_xy, max_z_err, max_tilt, max_yaw_err, yaw_span, failsafe_seen


def disarm(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 21196, 0, 0, 0, 0, 0,
    )


def main() -> int:
    args = parse_args()
    mav = mavutil.mavlink_connection(args.endpoint, autoreconnect=True)
    wait_heartbeat(mav, timeout=20)
    print(f"Connected system={mav.target_system} component={mav.target_component}", flush=True)
    start_gcs_heartbeat_thread(mav, period_s=0.5)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20.0)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20.0)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 5.0)
    latest_lpos = None
    latest_att = None
    try:
        set_param(mav, 'CST_POS_CTRL_EN', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'CST_POS_CTRL_TYP', 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_MODE_CMD', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_POS_ABS', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_POS_X', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Y', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Z', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_YAW', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=10)
        ground_lpos, ground_att = wait_for_ground_quiet(mav, duration_s=1.0, timeout=10.0,
                                       pos_window_xy=0.25, pos_window_z=0.25)
        baseline_x = float(ground_lpos.x)
        baseline_y = float(ground_lpos.y)
        baseline_z = float(ground_lpos.z)
        print(f'Yaw step target x={baseline_x:.3f} y={baseline_y:.3f} z={args.hover_z:.3f}', flush=True)
        arm_with_retries(mav, attempts=args.arm_attempts)
        time.sleep(max(0.5, args.pre_offboard_seconds))
        set_offboard(mav)
        latest_lpos, latest_att, *_ = execute_staged_hover_entry(
            mav,
            baseline_x=baseline_x,
            baseline_y=baseline_y,
            baseline_z=baseline_z,
            target_x=baseline_x,
            target_y=baseline_y,
            target_z=args.hover_z,
            yaw=0.0,
            ramp_seconds=args.ramp_seconds,
            direct_settle_seconds=args.settle_seconds,
            setpoint_period=args.setpoint_period,
            report_period=args.report_period,
            direct_xy_limit=args.xy_limit,
            direct_z_tolerance=args.z_tolerance,
            direct_tilt_limit_deg=args.tilt_limit_deg,
        )
        if latest_lpos is None or latest_att is None:
            raise RuntimeError('No hover state after staged entry')
        hover_yaw = float(latest_att.yaw)
        set_param(mav, 'TRJ_POS_ABS', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_POS_X', float(latest_lpos.x), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Y', float(latest_lpos.y), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Z', float(latest_lpos.z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_YAW', hover_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_ANCHOR_X', float(latest_lpos.x), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_ANCHOR_Y', float(latest_lpos.y), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_ANCHOR_Z', float(latest_lpos.z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'CST_POS_CTRL_EN', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print('custom_pos_control enabled', flush=True)
        latest_lpos, latest_att, *_ = observe_phase(
            mav, label='Custom hold', duration_s=args.custom_hold_seconds, report_period=args.report_period,
            ref_x=float(latest_lpos.x), ref_y=float(latest_lpos.y), ref_z=float(latest_lpos.z), yaw_ref=hover_yaw)
        target_yaw = wrap_pi(hover_yaw + args.yaw_step_rad)
        set_param(mav, 'TRJ_POS_YAW', target_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f'yaw step commanded target_yaw={math.degrees(target_yaw):.1f}deg', flush=True)
        latest_lpos, latest_att, max_xy, max_z_err, max_tilt, max_yaw_err, yaw_span, failsafe = observe_phase(
            mav, label='Yaw step', duration_s=args.yaw_step_seconds, report_period=args.report_period,
            ref_x=float(latest_lpos.x), ref_y=float(latest_lpos.y), ref_z=float(latest_lpos.z), yaw_ref=target_yaw)
        print(
            f'yaw_step_summary yaw_span={yaw_span:.3f} max_yaw_err={max_yaw_err:.3f} '
            f'max_xy={max_xy:.3f} max_z_err={max_z_err:.3f} max_tilt={max_tilt:.2f} failsafe={failsafe}',
            flush=True,
        )
        if yaw_span < args.yaw_success_rad:
            raise RuntimeError(f'Yaw step ineffective: yaw_span={yaw_span:.3f}rad < {args.yaw_success_rad:.3f}rad')
        return 0
    finally:
        try:
            set_param(mav, 'CST_POS_CTRL_EN', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, 'TRJ_MODE_CMD', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            disarm(mav)
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
