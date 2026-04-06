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


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Minimal baseline -> custom_pos_control -> trajectory_reader position-step handover test."
    )
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--hover-z", type=float, default=-3.0)
    p.add_argument("--ramp-seconds", type=float, default=8.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--direct-settle-seconds", type=float, default=2.0)
    p.add_argument("--custom-hold-seconds", type=float, default=15.0)
    p.add_argument("--trajectory-settle-timeout", type=float, default=10.0)
    p.add_argument("--trajectory-hold-seconds", type=float, default=5.0)
    p.add_argument("--step-x", type=float, default=0.20)
    p.add_argument("--step-y", type=float, default=0.0)
    p.add_argument("--step-z", type=float, default=0.0)
    p.add_argument("--setpoint-period", type=float, default=0.05)
    p.add_argument("--report-period", type=float, default=0.5)
    p.add_argument("--arm-attempts", type=int, default=3)
    p.add_argument("--direct-xy-limit", type=float, default=0.20)
    p.add_argument("--direct-z-tolerance", type=float, default=0.50)
    p.add_argument("--direct-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--custom-xy-limit", type=float, default=0.20)
    p.add_argument("--custom-z-tolerance", type=float, default=0.20)
    p.add_argument("--custom-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--trajectory-xy-limit", type=float, default=0.25)
    p.add_argument("--trajectory-z-tolerance", type=float, default=0.20)
    p.add_argument("--trajectory-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--trajectory-settle-radius", type=float, default=0.08)
    p.add_argument("--trajectory-settle-vxy", type=float, default=0.08)
    p.add_argument("--trajectory-settle-vz", type=float, default=0.08)
    p.add_argument("--min-step-progress", type=float, default=0.08)
    p.add_argument("--ground-xy-window", type=float, default=0.25)
    p.add_argument("--ground-z-window", type=float, default=0.25)
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


def observe_hold_phase(
    mav,
    *,
    label: str,
    duration_s: float,
    report_period: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    send_direct_setpoint: bool = False,
    yaw: float = 0.0,
) -> tuple[object, object, float, float, float, bool]:
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    termination_seen = False
    last_telemetry_time = None
    telemetry_timeout_s = 1.0

    while time.time() < end:
        if send_direct_setpoint:
            send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
        if not msg:
            if latest_lpos is not None and latest_att is not None and last_telemetry_time is not None:
                if time.time() - last_telemetry_time > telemetry_timeout_s:
                    raise RuntimeError(
                        f"Telemetry went stale during {label} (> {telemetry_timeout_s:.1f} s without "
                        "LOCAL_POSITION_NED/ATTITUDE)"
                    )
            continue
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Failsafe activated" in text or "Flight termination active" in text:
                failsafe_seen = True
            if "Flight termination active" in text:
                termination_seen = True
            continue
        if mtype == "LOCAL_POSITION_NED":
            latest_lpos = msg
            max_xy = max(max_xy, math.hypot(float(msg.x - ref_x), float(msg.y - ref_y)))
            max_z_err = max(max_z_err, abs(float(msg.z - ref_z)))
            last_telemetry_time = time.time()
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))
            last_telemetry_time = time.time()
        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"{label} x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    if latest_lpos is None or latest_att is None:
        raise RuntimeError(f"No LOCAL_POSITION_NED/ATTITUDE data during {label}")
    if last_telemetry_time is None or time.time() - last_telemetry_time > telemetry_timeout_s:
        raise RuntimeError(f"Telemetry went stale before {label} finished")
    return latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen


def observe_armed_ground_hold(
    mav,
    *,
    duration_s: float,
    report_period: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    yaw: float,
    setpoint_period: float,
    xy_limit: float = 0.12,
    z_tolerance: float = 0.12,
    speed_limit: float = 0.15,
    tilt_limit_deg: float = 5.0,
) -> None:
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_speed = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    termination_seen = False
    last_telemetry_time = None
    telemetry_timeout_s = 1.0

    while time.time() < end:
        send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)
        if not msg:
            if latest_lpos is not None and latest_att is not None and last_telemetry_time is not None:
                if time.time() - last_telemetry_time > telemetry_timeout_s:
                    raise RuntimeError(
                        f"Telemetry went stale during armed ground hold (> {telemetry_timeout_s:.1f} s without "
                        "LOCAL_POSITION_NED/ATTITUDE)"
                    )
            continue
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Failsafe activated" in text or "Flight termination active" in text:
                failsafe_seen = True
            if "Flight termination active" in text:
                termination_seen = True
            continue
        if mtype == "LOCAL_POSITION_NED":
            latest_lpos = msg
            max_xy = max(max_xy, math.hypot(float(msg.x - ref_x), float(msg.y - ref_y)))
            max_z_err = max(max_z_err, abs(float(msg.z - ref_z)))
            max_speed = max(max_speed, math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2))
            last_telemetry_time = time.time()
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))
            last_telemetry_time = time.time()
        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"Armed ground hold x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during armed ground hold")
    if last_telemetry_time is None or time.time() - last_telemetry_time > telemetry_timeout_s:
        raise RuntimeError("Telemetry went stale before armed ground hold finished")
    if (
        termination_seen
        or max_xy > xy_limit
        or max_z_err > z_tolerance
        or max_speed > speed_limit
        or max_tilt > tilt_limit_deg
    ):
        raise RuntimeError(
            f"Armed ground hold unstable: max_xy={max_xy:.3f}m max_z_err={max_z_err:.3f}m "
            f"max_speed={max_speed:.3f}m/s max_tilt={max_tilt:.2f}deg "
            f"failsafe={failsafe_seen} termination={termination_seen}"
        )


def execute_staged_hover_entry(
    mav,
    *,
    baseline_x: float,
    baseline_y: float,
    baseline_z: float,
    target_x: float,
    target_y: float,
    target_z: float,
    yaw: float,
    ramp_seconds: float,
    direct_settle_seconds: float,
    setpoint_period: float,
    report_period: float,
    direct_xy_limit: float,
    direct_z_tolerance: float,
    direct_tilt_limit_deg: float,
    max_stage_step_m: float | None = None,
    stage_hold_seconds: float = 0.0,
    lift_hold_seconds_override: float | None = None,
    allow_unstable_final_hover: bool = False,
) -> tuple[object, object, float, float, float, bool]:
    current_ref_x = target_x
    current_ref_y = target_y

    def _ramp_to_z(z_start: float, z_end: float, duration_s: float) -> None:
        t0 = time.time()
        ramp_end = t0 + max(0.1, duration_s)
        while time.time() < ramp_end:
            now = time.time()
            alpha = min(1.0, max(0.0, (now - t0) / max(0.1, duration_s)))
            z_sp = z_start + alpha * (z_end - z_start)
            send_position_target_local_ned(mav, current_ref_x, current_ref_y, z_sp, yaw)
            mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)

    def _hold_at_z(z_ref: float, duration_s: float) -> None:
        end = time.time() + max(0.0, duration_s)
        while time.time() < end:
            send_position_target_local_ned(mav, current_ref_x, current_ref_y, z_ref, yaw)
            mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)

    total_delta = target_z - baseline_z
    total_delta_abs = abs(total_delta)

    if total_delta_abs <= 0.6:
        lift_z = target_z
        lift_hold_seconds = 0.0
        lift_ramp_seconds = ramp_seconds
        final_ramp_seconds = 0.0
    else:
        lift_step = min(max(total_delta_abs * 0.35, 0.8), min(1.2, total_delta_abs))
        lift_z = baseline_z + math.copysign(lift_step, total_delta)
        lift_hold_seconds = min(2.0, max(1.0, direct_settle_seconds))
        lift_ramp_seconds = max(2.0, min(ramp_seconds * 0.45, ramp_seconds - 2.0))
        final_ramp_seconds = max(2.0, ramp_seconds - lift_ramp_seconds)

    if lift_hold_seconds_override is not None:
        lift_hold_seconds = max(0.0, float(lift_hold_seconds_override))

    print(
        f"Staged hover entry baseline_z={baseline_z:.3f} lift_z={lift_z:.3f} target_z={target_z:.3f}",
        flush=True,
    )

    print("Offboard ground hold", flush=True)
    _hold_at_z(baseline_z, 0.75)

    _ramp_to_z(baseline_z, lift_z, lift_ramp_seconds)

    if lift_hold_seconds > 0.0:
        lift_lpos, lift_att, lift_max_xy, lift_max_z_err, lift_max_tilt, lift_failsafe = observe_hold_phase(
            mav,
            label="Lift hover",
            duration_s=lift_hold_seconds,
            report_period=report_period,
            ref_x=target_x,
            ref_y=target_y,
            ref_z=lift_z,
            send_direct_setpoint=True,
            yaw=yaw,
        )
        lift_progress = abs(float(lift_lpos.z) - baseline_z)
        lift_final_xy = math.hypot(float(lift_lpos.x - target_x), float(lift_lpos.y - target_y))
        lift_final_z_err = abs(float(lift_lpos.z - lift_z))
        lift_final_tilt = math.degrees(max(abs(float(lift_att.roll)), abs(float(lift_att.pitch))))
        min_expected_lift = min(0.5, max(0.25, abs(lift_z - baseline_z) * 0.5))
        if (
            lift_failsafe
            or lift_max_xy > max(0.30, direct_xy_limit * 1.5)
            or lift_final_xy > max(0.20, direct_xy_limit)
            or lift_final_z_err > max(0.35, direct_z_tolerance)
            or lift_max_tilt > max(10.0, direct_tilt_limit_deg * 1.5)
            or lift_final_tilt > max(8.0, direct_tilt_limit_deg)
            or lift_progress < min_expected_lift
        ):
            raise RuntimeError(
                f"Lift hover unstable: max_xy={lift_max_xy:.3f}m max_z_err={lift_max_z_err:.3f}m "
                f"final_xy={lift_final_xy:.3f}m final_z_err={lift_final_z_err:.3f}m "
                f"max_tilt={lift_max_tilt:.2f}deg final_tilt={lift_final_tilt:.2f}deg "
                f"lift_progress={lift_progress:.3f}m "
                f"failsafe={lift_failsafe}"
            )

        current_ref_x = float(lift_lpos.x)
        current_ref_y = float(lift_lpos.y)
        print(
            f"Lift hover re-anchored x={current_ref_x:.3f} y={current_ref_y:.3f} z={float(lift_lpos.z):.3f}",
            flush=True,
        )

    if final_ramp_seconds > 0.0 and not math.isclose(lift_z, target_z, abs_tol=1e-6):
        stage_targets = [target_z]
        if max_stage_step_m is not None and max_stage_step_m > 0.0:
            delta_to_target = target_z - lift_z
            stage_count = max(1, math.ceil(abs(delta_to_target) / max_stage_step_m))
            if stage_count > 1:
                stage_targets = [
                    lift_z + (delta_to_target * (stage_idx / stage_count))
                    for stage_idx in range(1, stage_count + 1)
                ]

        prev_z = lift_z
        segment_ramp_seconds = max(1.5, final_ramp_seconds / max(1, len(stage_targets)))
        stage_hold = max(0.0, float(stage_hold_seconds))

        for stage_idx, stage_target_z in enumerate(stage_targets, start=1):
            if len(stage_targets) > 1:
                print(
                    f"Intermediate hover stage {stage_idx}/{len(stage_targets)} "
                    f"z={stage_target_z:.3f}",
                    flush=True,
                )
            _ramp_to_z(prev_z, stage_target_z, segment_ramp_seconds)
            if stage_hold > 0.0 and stage_idx < len(stage_targets):
                _hold_at_z(stage_target_z, stage_hold)
            prev_z = stage_target_z

    latest_lpos, latest_att, direct_max_xy, direct_max_z_err, direct_max_tilt, direct_failsafe = observe_hold_phase(
        mav,
        label="Direct hover",
        duration_s=direct_settle_seconds,
        report_period=report_period,
        ref_x=current_ref_x,
        ref_y=current_ref_y,
        ref_z=target_z,
        send_direct_setpoint=True,
        yaw=yaw,
    )
    direct_final_xy = math.hypot(float(latest_lpos.x - current_ref_x), float(latest_lpos.y - current_ref_y))
    direct_final_z_err = abs(float(latest_lpos.z - target_z))
    direct_final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
    if (
        direct_failsafe
        or direct_max_xy > direct_xy_limit
        or direct_final_xy > direct_xy_limit
        or direct_max_z_err > max(0.50, direct_z_tolerance * 2.5)
        or direct_final_z_err > direct_z_tolerance
        or direct_max_tilt > direct_tilt_limit_deg
        or direct_final_tilt > direct_tilt_limit_deg
    ):
        if allow_unstable_final_hover and not direct_failsafe:
            print(
                "Direct hover gate bypassed "
                f"(max_xy={direct_max_xy:.3f}m max_z_err={direct_max_z_err:.3f}m "
                f"final_xy={direct_final_xy:.3f}m final_z_err={direct_final_z_err:.3f}m "
                f"max_tilt={direct_max_tilt:.2f}deg final_tilt={direct_final_tilt:.2f}deg)",
                flush=True,
            )
            return latest_lpos, latest_att, direct_max_xy, direct_max_z_err, direct_max_tilt, direct_failsafe
        raise RuntimeError(
            f"Direct hover unstable: max_xy={direct_max_xy:.3f}m max_z_err={direct_max_z_err:.3f}m "
            f"final_xy={direct_final_xy:.3f}m final_z_err={direct_final_z_err:.3f}m "
            f"max_tilt={direct_max_tilt:.2f}deg final_tilt={direct_final_tilt:.2f}deg "
            f"failsafe={direct_failsafe}"
        )

    return latest_lpos, latest_att, direct_max_xy, direct_max_z_err, direct_max_tilt, direct_failsafe


def observe_trajectory_step(
    mav,
    *,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    start_x: float,
    start_y: float,
    start_z: float,
    settle_radius: float,
    settle_vxy: float,
    settle_vz: float,
    settle_timeout: float,
    hold_seconds: float,
    report_period: float,
) -> tuple[object, object, float, float, float, bool]:
    latest_lpos = None
    latest_att = None
    max_xy = 0.0
    max_tilt = 0.0
    max_progress = 0.0
    failsafe_seen = False
    settle_since = None
    next_report = 0.0
    deadline = time.time() + max(0.5, settle_timeout)

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
            dx = float(msg.x) - ref_x
            dy = float(msg.y) - ref_y
            dz = float(msg.z) - ref_z
            max_xy = max(max_xy, math.hypot(dx, dy))
            progress = math.sqrt(
                (float(msg.x) - start_x) ** 2
                + (float(msg.y) - start_y) ** 2
                + (float(msg.z) - start_z) ** 2
            )
            max_progress = max(max_progress, progress)
            pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
            speed_xy = math.hypot(float(msg.vx), float(msg.vy))
            speed_z = abs(float(msg.vz))
            if pos_err <= settle_radius and speed_xy <= settle_vxy and speed_z <= settle_vz:
                settle_since = settle_since or time.time()
            else:
                settle_since = None
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))

        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"Trajectory step x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f} "
                f"progress={max_progress:.3f}",
                flush=True,
            )
            next_report = now + report_period

        if settle_since and (time.time() - settle_since) >= hold_seconds:
            break

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during trajectory_reader phase")
    if not settle_since:
        raise RuntimeError("Trajectory step did not settle at the target")
    return latest_lpos, latest_att, max_xy, max_tilt, max_progress, failsafe_seen


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

        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart HIL clean before running trajectory_reader step."
            )

        yaw0 = 0.0
        for _ in range(10):
            msg = mav.recv_match(type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
            if not msg:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue
            yaw0 = float(msg.yaw)
            break

        target_x, target_y, target_z = baseline_x, baseline_y, baseline_z + args.hover_z
        print(
            f"Trajectory handover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} yaw={math.degrees(yaw0):.1f}deg",
            flush=True,
        )

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
        print("OFFBOARD accepted", flush=True)

        latest_lpos, latest_att, direct_max_xy, direct_max_z_err, direct_max_tilt, direct_failsafe = execute_staged_hover_entry(
            mav,
            baseline_x=baseline_x,
            baseline_y=baseline_y,
            baseline_z=baseline_z,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            yaw=yaw0,
            ramp_seconds=args.ramp_seconds,
            direct_settle_seconds=args.direct_settle_seconds,
            setpoint_period=args.setpoint_period,
            report_period=args.report_period,
            direct_xy_limit=args.direct_xy_limit,
            direct_z_tolerance=args.direct_z_tolerance,
            direct_tilt_limit_deg=args.direct_tilt_limit_deg,
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

        latest_lpos, latest_att, custom_max_xy, custom_max_z_err, custom_max_tilt, custom_failsafe = observe_hold_phase(
            mav,
            label="Custom hold",
            duration_s=args.custom_hold_seconds,
            report_period=args.report_period,
            ref_x=custom_ref_x,
            ref_y=custom_ref_y,
            ref_z=custom_ref_z,
        )

        custom_final_xy = math.hypot(float(latest_lpos.x) - custom_ref_x, float(latest_lpos.y) - custom_ref_y)
        custom_final_z_err = abs(float(latest_lpos.z) - custom_ref_z)

        if (
            custom_failsafe
            or custom_max_xy > args.custom_xy_limit
            or custom_max_z_err > args.custom_z_tolerance
            or custom_final_xy > args.custom_xy_limit
            or custom_final_z_err > args.custom_z_tolerance
            or custom_max_tilt > args.custom_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Custom hold unstable: max_xy={custom_max_xy:.3f}m final_xy={custom_final_xy:.3f}m "
                f"max_z_err={custom_max_z_err:.3f}m final_z_err={custom_final_z_err:.3f}m "
                f"max_tilt={custom_max_tilt:.2f}deg failsafe={custom_failsafe}"
            )

        handover_x = float(latest_lpos.x)
        handover_y = float(latest_lpos.y)
        handover_z = float(latest_lpos.z)
        handover_yaw = float(latest_att.yaw) if latest_att else yaw0

        traj_x = handover_x + args.step_x
        traj_y = handover_y + args.step_y
        traj_z = handover_z + args.step_z

        print(
            f"trajectory_reader target x={traj_x:.3f} y={traj_y:.3f} z={traj_z:.3f} yaw={math.degrees(handover_yaw):.1f}deg",
            flush=True,
        )

        set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", traj_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", traj_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", traj_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", handover_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("trajectory_reader position target updated", flush=True)

        latest_lpos, latest_att, traj_max_xy, traj_max_tilt, max_progress, traj_failsafe = observe_trajectory_step(
            mav,
            ref_x=traj_x,
            ref_y=traj_y,
            ref_z=traj_z,
            start_x=handover_x,
            start_y=handover_y,
            start_z=handover_z,
            settle_radius=args.trajectory_settle_radius,
            settle_vxy=args.trajectory_settle_vxy,
            settle_vz=args.trajectory_settle_vz,
            settle_timeout=args.trajectory_settle_timeout,
            hold_seconds=args.trajectory_hold_seconds,
            report_period=args.report_period,
        )

        final_xy = math.hypot(float(latest_lpos.x) - traj_x, float(latest_lpos.y) - traj_y)
        final_z_err = abs(float(latest_lpos.z) - traj_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        print(
            f"Trajectory reader summary: progress={max_progress:.3f}m max_xy={traj_max_xy:.3f}m "
            f"max_tilt={traj_max_tilt:.2f}deg final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m "
            f"failsafe={traj_failsafe}",
            flush=True,
        )

        if (
            traj_failsafe
            or max_progress < args.min_step_progress
            or traj_max_xy > args.trajectory_xy_limit
            or final_xy > args.trajectory_settle_radius
            or final_z_err > args.trajectory_z_tolerance
            or traj_max_tilt > args.trajectory_tilt_limit_deg
            or final_tilt > args.trajectory_tilt_limit_deg
        ):
            raise RuntimeError(
                f"trajectory_reader position step unstable: progress={max_progress:.3f}m max_xy={traj_max_xy:.3f}m "
                f"final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m "
                f"max_tilt={traj_max_tilt:.2f}deg final_tilt={final_tilt:.2f}deg failsafe={traj_failsafe}"
            )

        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3 << 16)
        disarm(mav)
        print("trajectory_reader position-step stable", flush=True)
        return 0
    finally:
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
