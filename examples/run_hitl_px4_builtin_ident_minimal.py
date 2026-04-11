#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (SCRIPT_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from experimental_validation.hitl_catalog import IDENTIFICATION_PROFILES, identification_duration_s
from run_hitl_udp_sequence import (
    arm_with_retries,
    decode_statustext,
    request_message_interval,
    robust_ground_baseline,
    send_nav_takeoff,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
    wait_for_local_position,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_builtin_trajectory_minimal import disarm
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    execute_staged_hover_entry,
    observe_armed_ground_hold,
    observe_hold_phase,
    send_position_target_local_ned,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Minimal baseline -> custom_pos hold -> built-in identification profile."
    )
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--profile", choices=sorted(IDENTIFICATION_PROFILES), default="hover_thrust")
    p.add_argument("--hover-z", type=float, default=-5.0)
    p.add_argument("--ramp-seconds", type=float, default=8.0)
    p.add_argument("--takeoff-timeout", type=float, default=45.0)
    p.add_argument("--takeoff-settle-seconds", type=float, default=2.0)
    p.add_argument("--takeoff-stable-window", type=float, default=2.0)
    p.add_argument("--takeoff-z-tolerance", type=float, default=0.25)
    p.add_argument("--takeoff-max-horiz-speed", type=float, default=0.15)
    p.add_argument("--takeoff-max-vert-speed", type=float, default=0.10)
    p.add_argument("--takeoff-tilt-limit-deg", type=float, default=5.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--direct-settle-seconds", type=float, default=1.5)
    p.add_argument("--custom-handover-seconds", type=float, default=1.5)
    p.add_argument("--custom-hold-seconds", type=float, default=3.0)
    p.add_argument("--ident-tail-seconds", type=float, default=4.0)
    p.add_argument("--setpoint-period", type=float, default=0.05)
    p.add_argument("--report-period", type=float, default=0.5)
    p.add_argument("--arm-attempts", type=int, default=3)
    p.add_argument("--direct-xy-limit", type=float, default=0.20)
    p.add_argument("--direct-z-tolerance", type=float, default=0.50)
    p.add_argument("--direct-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--custom-xy-limit", type=float, default=0.20)
    p.add_argument("--custom-z-tolerance", type=float, default=0.20)
    p.add_argument("--custom-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--ident-xy-envelop-limit", type=float, default=6.0)
    p.add_argument("--ident-z-tolerance", type=float, default=1.5)
    p.add_argument("--ident-tilt-limit-deg", type=float, default=45.0)
    p.add_argument("--ground-xy-window", type=float, default=0.25)
    p.add_argument("--ground-z-window", type=float, default=0.25)
    p.add_argument("--land-timeout", type=float, default=45.0)
    return p.parse_args()


def identification_relative_position(profile: str, t_s: float) -> tuple[float, float, float]:
    duration_s = identification_duration_s(profile)

    def identification_window01(u: float) -> float:
        clamped = max(0.0, min(1.0, u))
        s = math.sin(math.pi * clamped)
        return s * s

    def windowed_multisine(amplitudes: tuple[float, ...], cycles: tuple[float, ...]) -> float:
        if duration_s <= 0.0:
            return 0.0
        u = max(0.0, min(1.0, t_s / duration_s))
        window = identification_window01(u)
        value = 0.0
        for amplitude, cycle in zip(amplitudes, cycles):
            value += amplitude * math.sin(2.0 * math.pi * cycle * u)
        return window * value

    def integrate_velocity_transition(tau_s: float, segment_s: float, v0: float, v1: float) -> float:
        if segment_s <= 1e-4:
            return 0.5 * (v0 + v1) * max(tau_s, 0.0)
        tau = max(0.0, min(segment_s, tau_s))
        return v0 * tau + (v1 - v0) * (0.5 * tau - segment_s / (2.0 * math.pi) * math.sin(math.pi * tau / segment_s))

    def symmetric_velocity_cycle_position(ramp_s: float, hold_s: float, dwell_s: float, cruise_v: float) -> float:
        seg_accel_pos_end = ramp_s
        seg_hold_pos_end = seg_accel_pos_end + hold_s
        seg_decel_pos_end = seg_hold_pos_end + ramp_s
        seg_dwell_end = seg_decel_pos_end + dwell_s
        seg_accel_neg_end = seg_dwell_end + ramp_s
        seg_hold_neg_end = seg_accel_neg_end + hold_s
        seg_decel_neg_end = seg_hold_neg_end + ramp_s
        distance_positive = cruise_v * (ramp_s + hold_s)
        t = max(t_s, 0.0)

        if t < seg_accel_pos_end:
            return integrate_velocity_transition(t, ramp_s, 0.0, cruise_v)
        if t < seg_hold_pos_end:
            return 0.5 * cruise_v * ramp_s + cruise_v * (t - seg_accel_pos_end)
        if t < seg_decel_pos_end:
            tau = t - seg_hold_pos_end
            return 0.5 * cruise_v * ramp_s + cruise_v * hold_s + integrate_velocity_transition(tau, ramp_s, cruise_v, 0.0)
        if t < seg_dwell_end:
            return distance_positive
        if t < seg_accel_neg_end:
            tau = t - seg_dwell_end
            return distance_positive + integrate_velocity_transition(tau, ramp_s, 0.0, -cruise_v)
        if t < seg_hold_neg_end:
            return distance_positive - 0.5 * cruise_v * ramp_s - cruise_v * (t - seg_accel_neg_end)
        if t < seg_decel_neg_end:
            tau = t - seg_hold_neg_end
            return 0.5 * cruise_v * ramp_s + integrate_velocity_transition(tau, ramp_s, -cruise_v, 0.0)
        return 0.0

    def leveled_step_sequence_position(segment_s: float, sequence: tuple[float, ...]) -> float:
        idx = max(0, min(int(math.floor(max(t_s, 0.0) / segment_s)), len(sequence) - 1))
        return sequence[idx]

    if profile == "hover_thrust":
        return (
            0.0,
            0.0,
            windowed_multisine((0.24, 0.10, 0.05), (2.0, 4.0, 6.0)),
        )
    if profile == "roll_sweep":
        return (
            0.0,
            windowed_multisine((0.42, 0.18, 0.08), (1.5, 3.0, 5.0)),
            0.0,
        )
    if profile == "pitch_sweep":
        return (
            windowed_multisine((0.42, 0.18, 0.08), (1.5, 3.0, 5.0)),
            0.0,
            0.0,
        )
    if profile == "yaw_sweep":
        return (0.0, 0.0, 0.0)
    if profile == "drag_x":
        return (symmetric_velocity_cycle_position(2.5, 1.5, 1.0, 0.60), 0.0, 0.0)
    if profile == "drag_y":
        return (0.0, symmetric_velocity_cycle_position(2.5, 1.5, 1.0, 0.60), 0.0)
    if profile == "drag_z":
        return (0.0, 0.0, symmetric_velocity_cycle_position(1.8, 1.0, 0.8, 0.32))
    if profile == "mass_vertical":
        return (
            0.0,
            0.0,
            windowed_multisine((0.34, 0.18, 0.10), (2.0, 4.0, 6.0)),
        )
    if profile == "motor_step":
        sequence = (0.00, 0.05, 0.00, -0.03, 0.00, 0.08, 0.00, -0.05, 0.00, 0.03)
        return (0.0, 0.0, leveled_step_sequence_position(1.4, sequence))
    if profile == "actuator_lag_collective":
        sequence = (0.00, 0.04, -0.04, 0.06, -0.06, 0.08, -0.08, 0.10, -0.10, 0.05, -0.05, 0.00)
        return (0.0, 0.0, leveled_step_sequence_position(1.0, sequence))
    if profile == "bridge_probe_xy":
        def shifted_cycle(start_s: float) -> float:
            ramp_s = 1.0
            hold_s = 0.75
            dwell_s = 0.5
            cycle_duration_s = 4.0 * ramp_s + 2.0 * hold_s + dwell_s
            if t_s < start_s or t_s >= start_s + cycle_duration_s:
                return 0.0
            local_t = t_s - start_s

            seg_accel_pos_end = ramp_s
            seg_hold_pos_end = seg_accel_pos_end + hold_s
            seg_decel_pos_end = seg_hold_pos_end + ramp_s
            seg_dwell_end = seg_decel_pos_end + dwell_s
            seg_accel_neg_end = seg_dwell_end + ramp_s
            seg_hold_neg_end = seg_accel_neg_end + hold_s
            seg_decel_neg_end = seg_hold_neg_end + ramp_s
            cruise_v = 0.70
            distance_positive = cruise_v * (ramp_s + hold_s)

            if local_t < seg_accel_pos_end:
                return integrate_velocity_transition(local_t, ramp_s, 0.0, cruise_v)
            if local_t < seg_hold_pos_end:
                return 0.5 * cruise_v * ramp_s + cruise_v * (local_t - seg_accel_pos_end)
            if local_t < seg_decel_pos_end:
                tau = local_t - seg_hold_pos_end
                return 0.5 * cruise_v * ramp_s + cruise_v * hold_s + integrate_velocity_transition(tau, ramp_s, cruise_v, 0.0)
            if local_t < seg_dwell_end:
                return distance_positive
            if local_t < seg_accel_neg_end:
                tau = local_t - seg_dwell_end
                return distance_positive + integrate_velocity_transition(tau, ramp_s, 0.0, -cruise_v)
            if local_t < seg_hold_neg_end:
                return distance_positive - 0.5 * cruise_v * ramp_s - cruise_v * (local_t - seg_accel_neg_end)
            if local_t < seg_decel_neg_end:
                tau = local_t - seg_hold_neg_end
                return 0.5 * cruise_v * ramp_s + integrate_velocity_transition(tau, ramp_s, -cruise_v, 0.0)
            return 0.0

        return (
            shifted_cycle(0.0),
            shifted_cycle(6.0),
            0.0,
        )
    return (0.0, 0.0, 0.0)


def observe_ident(
    mav,
    *,
    duration_s: float,
    profile: str,
    anchor_x: float,
    anchor_y: float,
    anchor_z: float,
    ref_z: float,
    report_period: float,
) -> tuple[object, object, float, float, float, bool]:
    t0 = time.time()
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    last_telemetry_time = None
    telemetry_timeout_s = 1.0

    while time.time() < end:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
        if not msg:
            if latest_lpos is not None and latest_att is not None and last_telemetry_time is not None:
                if time.time() - last_telemetry_time > telemetry_timeout_s:
                    raise RuntimeError(
                        f"Telemetry went stale during identification phase (> {telemetry_timeout_s:.1f} s without "
                        "LOCAL_POSITION_NED/ATTITUDE)"
                    )
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
            ident_t = min(max(0.0, time.time() - t0), identification_duration_s(profile))
            rel_x, rel_y, rel_z = identification_relative_position(profile, ident_t)
            ref_x_now = anchor_x + rel_x
            ref_y_now = anchor_y + rel_y
            ref_z_now = anchor_z + rel_z
            max_xy = max(max_xy, math.hypot(float(msg.x - ref_x_now), float(msg.y - ref_y_now)))
            max_z_err = max(max_z_err, abs(float(msg.z - ref_z_now)))
            last_telemetry_time = time.time()
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))
            last_telemetry_time = time.time()
        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"Identification x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during identification phase")
    if last_telemetry_time is None or time.time() - last_telemetry_time > telemetry_timeout_s:
        raise RuntimeError("Telemetry went stale before identification phase finished")
    return latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen


def best_effort_set_param(mav, name: str, value: float | int, param_type: int) -> None:
    try:
        set_param(mav, name, value, param_type)
    except Exception as exc:
        print(f"Warning: best-effort param reset failed for {name}={value}: {exc}", flush=True)


def best_effort_disarm(mav) -> None:
    try:
        disarm(mav)
    except Exception as exc:
        print(f"Warning: best-effort disarm failed: {exc}", flush=True)


def set_position_mode(mav) -> None:
    mav.set_mode_px4("POSCTL", 0, 0)


def set_land_mode(mav) -> None:
    mav.set_mode_px4("LAND", 0, 0)


def wait_for_takeoff_hover(
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

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        if msg_type == "LOCAL_POSITION_NED":
            latest_lpos = msg
        elif msg_type == "ATTITUDE":
            latest_att = msg

        now = time.time()
        if latest_lpos and latest_att and now >= next_report:
            print(
                f"Takeoff hover x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if latest_lpos is None or latest_att is None:
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

    raise RuntimeError(
        f"Takeoff hover did not settle near z={target_z:.3f}; last_lpos={latest_lpos} last_att={latest_att}"
    )


def observe_direct_offboard_hold(
    mav,
    *,
    duration_s: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    yaw: float,
    setpoint_period: float,
    report_period: float,
) -> tuple[object, object, float, float, float, bool]:
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    last_telemetry_time = None
    telemetry_timeout_s = 1.0

    while time.time() < end:
        send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)
        if not msg:
            if latest_lpos is not None and latest_att is not None and last_telemetry_time is not None:
                if time.time() - last_telemetry_time > telemetry_timeout_s:
                    raise RuntimeError("Telemetry went stale during direct offboard hold")
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Failsafe activated" in text or "Flight termination active" in text:
                failsafe_seen = True
            continue
        if msg_type == "LOCAL_POSITION_NED":
            latest_lpos = msg
            max_xy = max(max_xy, math.hypot(float(msg.x) - ref_x, float(msg.y) - ref_y))
            max_z_err = max(max_z_err, abs(float(msg.z) - ref_z))
            last_telemetry_time = time.time()
        elif msg_type == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))
            last_telemetry_time = time.time()

        now = time.time()
        if latest_lpos and latest_att and now >= next_report:
            print(
                f"Direct hover x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during direct offboard hold")
    return latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe_seen


def wait_until_landed(mav, *, ground_z: float, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    latest = None
    quiet_since = None
    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        if msg_type == "HEARTBEAT":
            base_mode = int(getattr(msg, "base_mode", 0))
            if not (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
            continue
        latest = msg
        near_ground = abs(float(msg.z) - ground_z) <= 0.25
        slow = math.hypot(float(msg.vx), float(msg.vy)) <= 0.15 and abs(float(msg.vz)) <= 0.15
        if near_ground and slow:
            quiet_since = quiet_since or time.time()
            if time.time() - quiet_since >= 2.0:
                return
        else:
            quiet_since = None

    raise RuntimeError(f"Vehicle did not land in time; last_local_position={latest}")


def main() -> int:
    args = parse_args()

    if args.profile == "yaw_sweep":
        args.direct_xy_limit = max(args.direct_xy_limit, 1.50)
        args.direct_z_tolerance = max(args.direct_z_tolerance, 0.80)
        args.direct_tilt_limit_deg = max(args.direct_tilt_limit_deg, 35.0)
        args.direct_settle_seconds = max(args.direct_settle_seconds, 4.0)
        args.custom_xy_limit = max(args.custom_xy_limit, 0.35)
        args.custom_z_tolerance = max(args.custom_z_tolerance, 0.10)
        args.custom_tilt_limit_deg = max(args.custom_tilt_limit_deg, 20.0)

    kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    successful_completion = False
    baseline_z = 0.0

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
        set_param(mav, "MIS_TAKEOFF_ALT", max(0.5, abs(float(args.hover_z))), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart HIL clean before running built-in identification."
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
            f"Built-in identification hover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} "
            f"yaw={math.degrees(yaw0):.1f}deg profile={args.profile}",
            flush=True,
        )

        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        observe_armed_ground_hold(
            mav,
            duration_s=1.0,
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
            direct_settle_seconds=max(0.5, args.direct_settle_seconds),
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
        set_param(mav, "TRJ_ANCHOR_X", float(latest_lpos.x), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_ANCHOR_Y", float(latest_lpos.y), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_ANCHOR_Z", float(latest_lpos.z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("custom_pos_control enabled", flush=True)

        custom_ref_x = float(latest_lpos.x)
        custom_ref_y = float(latest_lpos.y)
        custom_ref_z = float(latest_lpos.z)
        custom_ref_yaw = float(latest_att.yaw)

        handover_until = time.monotonic() + max(0.0, args.custom_handover_seconds)
        while time.monotonic() < handover_until:
            send_position_target_local_ned(mav, custom_ref_x, custom_ref_y, custom_ref_z, custom_ref_yaw)
            time.sleep(args.setpoint_period)

        latest_lpos, latest_att, custom_max_xy, custom_max_z_err, custom_max_tilt, custom_failsafe = observe_hold_phase(
            mav,
            label="Custom hold",
            duration_s=args.custom_hold_seconds,
            report_period=args.report_period,
            ref_x=custom_ref_x,
            ref_y=custom_ref_y,
            ref_z=custom_ref_z,
        )
        if (
            custom_failsafe
            or custom_max_xy > args.custom_xy_limit
            or custom_max_z_err > args.custom_z_tolerance
            or custom_max_tilt > args.custom_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Custom hold unstable: max_xy={custom_max_xy:.3f}m max_z_err={custom_max_z_err:.3f}m "
                f"max_tilt={custom_max_tilt:.2f}deg failsafe={custom_failsafe}"
            )

        set_param(mav, "CST_POS_CTRL_TYP", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_IDENT_PROF", IDENTIFICATION_PROFILES[args.profile], mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print(f"identification profile {args.profile} started", flush=True)
        set_param(mav, "TRJ_MODE_CMD", 2, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        latest_lpos, latest_att, ident_max_xy, ident_max_z_err, ident_max_tilt, ident_failsafe = observe_ident(
            mav,
            duration_s=identification_duration_s(args.profile) + args.ident_tail_seconds,
            profile=args.profile,
            anchor_x=custom_ref_x,
            anchor_y=custom_ref_y,
            anchor_z=custom_ref_z,
            ref_z=custom_ref_z,
            report_period=args.report_period,
        )

        rel_x_end, rel_y_end, rel_z_end = identification_relative_position(args.profile, identification_duration_s(args.profile))
        final_ref_x = custom_ref_x + rel_x_end
        final_ref_y = custom_ref_y + rel_y_end
        final_ref_z = custom_ref_z + rel_z_end
        final_xy = math.hypot(float(latest_lpos.x) - final_ref_x, float(latest_lpos.y) - final_ref_y)
        final_z_err = abs(float(latest_lpos.z) - final_ref_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        print(
            f"Built-in identification summary: profile={args.profile} max_xy={ident_max_xy:.3f}m "
            f"max_z_err={ident_max_z_err:.3f}m max_tilt={ident_max_tilt:.2f}deg "
            f"final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m final_tilt={final_tilt:.2f}deg "
            f"failsafe={ident_failsafe}",
            flush=True,
        )

        if (
            ident_failsafe
            or ident_max_xy > args.ident_xy_envelop_limit
            or ident_max_z_err > args.ident_z_tolerance
            or ident_max_tilt > args.ident_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Built-in identification unstable: max_xy={ident_max_xy:.3f}m max_z_err={ident_max_z_err:.3f}m "
                f"max_tilt={ident_max_tilt:.2f}deg failsafe={ident_failsafe}"
            )

        print("built-in identification stable", flush=True)
        best_effort_set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        best_effort_set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        best_effort_set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try:
            set_position_mode(mav)
            time.sleep(1.0)
        except Exception as exc:
            print(f"Warning: best-effort POSCTL restore failed: {exc}", flush=True)
        try:
            set_land_mode(mav)
            wait_until_landed(mav, ground_z=baseline_z, timeout_s=args.land_timeout)
        except Exception as exc:
            print(f"Warning: best-effort landing failed: {exc}", flush=True)
            best_effort_disarm(mav)
        successful_completion = True
        return 0
    finally:
        if not successful_completion:
            best_effort_set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            best_effort_set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            best_effort_set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            try:
                set_position_mode(mav)
                time.sleep(1.0)
                set_land_mode(mav)
                wait_until_landed(mav, ground_z=baseline_z, timeout_s=args.land_timeout)
            except Exception:
                best_effort_disarm(mav)
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
