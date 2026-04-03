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
    set_param,
    set_offboard,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    execute_staged_hover_entry,
    observe_armed_ground_hold,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Minimal baseline PX4 HIL hover test without parameter overrides.')
    p.add_argument('--endpoint', default='udpin:127.0.0.1:14550')
    p.add_argument('--baud', type=int, default=57600)
    p.add_argument('--hover-z', type=float, default=-3.0)
    p.add_argument('--ramp-seconds', type=float, default=8.0)
    p.add_argument('--hold-seconds', type=float, default=8.0)
    p.add_argument('--pre-offboard-seconds', type=float, default=2.0)
    p.add_argument('--setpoint-period', type=float, default=0.05)
    p.add_argument('--report-period', type=float, default=0.5)
    p.add_argument('--arm-attempts', type=int, default=3)
    p.add_argument('--xy-limit', type=float, default=0.20)
    p.add_argument('--z-tolerance', type=float, default=0.20)
    p.add_argument('--tilt-limit-deg', type=float, default=6.0)
    p.add_argument('--ground-xy-window', type=float, default=0.25)
    p.add_argument('--ground-z-window', type=float, default=0.25)
    return p.parse_args()


def disarm(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        21196,
        0, 0, 0, 0, 0,
    )


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


def main() -> int:
    args = parse_args()
    kwargs = {'autoreconnect': False}
    if args.endpoint.startswith('/dev/'):
        kwargs['baud'] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)

    try:
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)
        set_param(mav, 'CST_POS_CTRL_EN', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'CST_POS_CTRL_TYP', 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_MODE_CMD', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_POS_ABS', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, 'TRJ_POS_X', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Y', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_Z', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, 'TRJ_POS_YAW', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart HIL clean before running baseline hover."
            )

        yaw0 = 0.0
        for _ in range(10):
            msg = mav.recv_match(type=['ATTITUDE', 'STATUSTEXT'], blocking=True, timeout=0.5)
            if not msg:
                continue
            if msg.get_type() == 'STATUSTEXT':
                print(decode_statustext(msg), flush=True)
                continue
            yaw0 = float(msg.yaw)
            break

        target_x, target_y, target_z = baseline_x, baseline_y, baseline_z + args.hover_z
        print(f'Minimal hover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} yaw={math.degrees(yaw0):.1f}deg', flush=True)

        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        observe_armed_ground_hold(
            mav,
            duration_s=max(0.5, args.pre_offboard_seconds),
            report_period=args.report_period,
            ref_x=target_x,
            ref_y=target_y,
            ref_z=baseline_z,
            yaw=yaw0,
            setpoint_period=args.setpoint_period,
        )

        set_offboard(mav)
        print('OFFBOARD accepted', flush=True)

        latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen = execute_staged_hover_entry(
            mav,
            baseline_x=baseline_x,
            baseline_y=baseline_y,
            baseline_z=baseline_z,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            yaw=yaw0,
            ramp_seconds=args.ramp_seconds,
            direct_settle_seconds=args.hold_seconds,
            setpoint_period=args.setpoint_period,
            report_period=args.report_period,
            direct_xy_limit=args.xy_limit,
            direct_z_tolerance=args.z_tolerance,
            direct_tilt_limit_deg=args.tilt_limit_deg,
        )

        if latest_lpos is None or latest_att is None:
            raise RuntimeError('No LOCAL_POSITION_NED/ATTITUDE data during baseline hover')

        final_xy = math.hypot(float(latest_lpos.x) - target_x, float(latest_lpos.y) - target_y)
        final_z_err = abs(float(latest_lpos.z) - target_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        print(
            f"Baseline hover summary: max_xy={max_xy:.3f}m max_z_err={max_z_err:.3f}m "
            f"max_tilt={max_tilt:.2f}deg final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m "
            f"final_tilt={final_tilt:.2f}deg failsafe={failsafe_seen}",
            flush=True,
        )

        if (
            failsafe_seen
            or max_xy > args.xy_limit
            or max_tilt > args.tilt_limit_deg
            or final_xy > args.xy_limit
            or max_z_err > max(0.50, args.z_tolerance * 2.5)
            or final_z_err > args.z_tolerance
            or final_tilt > args.tilt_limit_deg
        ):
            raise RuntimeError(
                f"Baseline hover unstable: max_xy={max_xy:.3f}m final_xy={final_xy:.3f}m "
                f"max_z_err={max_z_err:.3f}m final_z_err={final_z_err:.3f}m "
                f"max_tilt={max_tilt:.2f}deg final_tilt={final_tilt:.2f}deg failsafe={failsafe_seen}"
            )

        set_position_mode = 3 << 16
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, set_position_mode)
        disarm(mav)
        print('Minimal baseline run complete', flush=True)
        return 0
    finally:
        try:
            set_param(mav, 'CST_POS_CTRL_EN', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        except Exception:
            pass
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == '__main__':
    raise SystemExit(main())
