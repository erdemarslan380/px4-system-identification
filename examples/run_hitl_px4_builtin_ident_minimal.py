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
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
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
    p.add_argument("--hover-z", type=float, default=-3.0)
    p.add_argument("--ramp-seconds", type=float, default=8.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--direct-settle-seconds", type=float, default=2.0)
    p.add_argument("--custom-handover-seconds", type=float, default=2.0)
    p.add_argument("--custom-hold-seconds", type=float, default=5.0)
    p.add_argument("--ident-tail-seconds", type=float, default=8.0)
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
    return p.parse_args()


def identification_relative_position(profile: str, t_s: float) -> tuple[float, float, float]:
    two_pi = 2.0 * math.pi
    phase_s = 0.0
    if profile == "roll_sweep":
        phase_s = 6.4080236
    elif profile == "pitch_sweep":
        phase_s = 6.457
    if profile == "hover_thrust":
        return (
            0.0,
            0.0,
            0.35 * math.sin(two_pi * 0.12 * t_s) + 0.18 * math.sin(two_pi * 0.27 * t_s),
        )
    if profile == "roll_sweep":
        s0 = (
            0.60 * math.sin(two_pi * 0.09 * phase_s)
            + 0.28 * math.sin(two_pi * 0.19 * phase_s)
            + 0.12 * math.sin(two_pi * 0.31 * phase_s)
        )
        s = (
            0.60 * math.sin(two_pi * 0.09 * (t_s + phase_s))
            + 0.28 * math.sin(two_pi * 0.19 * (t_s + phase_s))
            + 0.12 * math.sin(two_pi * 0.31 * (t_s + phase_s))
        )
        return (
            0.0,
            s - s0,
            0.0,
        )
    if profile == "pitch_sweep":
        s0 = (
            0.60 * math.sin(two_pi * 0.09 * phase_s)
            + 0.28 * math.sin(two_pi * 0.19 * phase_s)
            + 0.12 * math.sin(two_pi * 0.31 * phase_s)
        )
        s = (
            0.60 * math.sin(two_pi * 0.09 * (t_s + phase_s))
            + 0.28 * math.sin(two_pi * 0.19 * (t_s + phase_s))
            + 0.12 * math.sin(two_pi * 0.31 * (t_s + phase_s))
        )
        return (
            s - s0,
            0.0,
            0.0,
        )
    if profile == "yaw_sweep":
        return (0.0, 0.0, 0.0)
    if profile == "drag_x":
        return (1.20 * math.sin(two_pi * 0.08 * t_s), 0.0, 0.0)
    if profile == "drag_y":
        return (0.0, 1.20 * math.sin(two_pi * 0.08 * t_s), 0.0)
    if profile == "drag_z":
        return (0.0, 0.0, 0.50 * math.sin(two_pi * 0.09 * t_s))
    if profile == "mass_vertical":
        return (
            0.0,
            0.0,
            0.55 * math.sin(two_pi * 0.08 * t_s)
            + 0.35 * math.sin(two_pi * 0.17 * t_s)
            + 0.18 * math.sin(two_pi * 0.29 * t_s)
            + 0.10 * math.sin(two_pi * 0.41 * t_s),
        )
    if profile == "motor_step":
        sequence = (0.00, 0.06, -0.02, 0.11, -0.04, 0.16, -0.06, 0.00)
        segment = max(0, min(int(math.floor(t_s / 2.5)), len(sequence) - 1))
        return (0.0, 0.0, sequence[segment])
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

        handover_until = time.monotonic() + max(0.0, args.custom_handover_seconds)
        while time.monotonic() < handover_until:
            send_position_target_local_ned(mav, custom_ref_x, custom_ref_y, custom_ref_z, float(latest_att.yaw))
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
        successful_completion = True
        return 0
    finally:
        if not successful_completion:
            best_effort_set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            best_effort_set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            best_effort_set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
