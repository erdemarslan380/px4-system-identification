#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
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

from experimental_validation.hitl_catalog import CAMPAIGNS, IDENTIFICATION_PROFILES, identification_duration_s
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
from run_hitl_px4_builtin_ident_minimal import observe_ident
from run_hitl_px4_builtin_trajectory_minimal import TRAJ_DURATION_S, observe_builtin_trajectory
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    observe_hold_phase,
    send_position_target_local_ned,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run built-in HIL identification and trajectory segments sequentially in a single session.")
    ap.add_argument("--campaign", choices=sorted(CAMPAIGNS), default="full_stack")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--hover-z", type=float, default=-3.0)
    ap.add_argument("--ramp-seconds", type=float, default=8.0)
    ap.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    ap.add_argument("--direct-settle-seconds", type=float, default=2.0)
    ap.add_argument("--custom-hold-seconds", type=float, default=15.0)
    ap.add_argument("--between-segments-hold-seconds", type=float, default=4.0)
    ap.add_argument("--ident-tail-seconds", type=float, default=8.0)
    ap.add_argument("--trajectory-tail-seconds", type=float, default=10.0)
    ap.add_argument("--setpoint-period", type=float, default=0.05)
    ap.add_argument("--report-period", type=float, default=0.5)
    ap.add_argument("--arm-attempts", type=int, default=3)
    ap.add_argument("--ground-xy-window", type=float, default=0.25)
    ap.add_argument("--ground-z-window", type=float, default=0.25)
    ap.add_argument("--direct-xy-limit", type=float, default=0.20)
    ap.add_argument("--direct-z-tolerance", type=float, default=0.50)
    ap.add_argument("--direct-tilt-limit-deg", type=float, default=6.0)
    ap.add_argument("--custom-xy-limit", type=float, default=0.20)
    ap.add_argument("--custom-z-tolerance", type=float, default=0.20)
    ap.add_argument("--custom-tilt-limit-deg", type=float, default=6.0)
    ap.add_argument("--output-json", default="")
    return ap.parse_args()


def best_effort_param_set(mav, name: str, value: float | int, param_type: int) -> None:
    try:
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            name.encode("ascii").ljust(16, b"\0"),
            float(value),
            param_type,
        )
    except Exception as exc:
        print(f"Warning: failed to send best-effort PARAM_SET {name}={value}: {exc}", flush=True)


def best_effort_return_to_custom_hold(mav, *, hold_x: float, hold_y: float, hold_z: float, hold_yaw: float) -> None:
    best_effort_param_set(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    best_effort_param_set(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    best_effort_param_set(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    best_effort_param_set(mav, "TRJ_POS_X", hold_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    best_effort_param_set(mav, "TRJ_POS_Y", hold_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    best_effort_param_set(mav, "TRJ_POS_Z", hold_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    best_effort_param_set(mav, "TRJ_POS_YAW", hold_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def settle_custom_hold(mav, *, ref_x: float, ref_y: float, ref_z: float, duration_s: float, report_period: float) -> tuple[object, object]:
    latest_lpos, latest_att, *_ = observe_hold_phase(
        mav,
        label="Recovery hold",
        duration_s=duration_s,
        report_period=report_period,
        ref_x=ref_x,
        ref_y=ref_y,
        ref_z=ref_z,
    )
    return latest_lpos, latest_att


def ensure_stable_hold(
    *,
    label: str,
    max_xy: float,
    max_z_err: float,
    max_tilt: float,
    failsafe: bool,
    xy_limit: float,
    z_tolerance: float,
    tilt_limit_deg: float,
) -> None:
    if failsafe:
        raise RuntimeError(f"{label} entered failsafe")
    if max_xy > xy_limit:
        raise RuntimeError(f"{label} unstable: max_xy={max_xy:.3f} m > {xy_limit:.3f} m")
    if max_z_err > z_tolerance:
        raise RuntimeError(f"{label} unstable: max_z_err={max_z_err:.3f} m > {z_tolerance:.3f} m")
    if max_tilt > tilt_limit_deg:
        raise RuntimeError(f"{label} unstable: max_tilt={max_tilt:.2f} deg > {tilt_limit_deg:.2f} deg")


def main() -> int:
    args = parse_args()
    kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        kwargs["baud"] = args.baud

    output_json = Path(args.output_json).expanduser().resolve() if args.output_json else None
    summary: dict[str, object] = {
        "campaign": args.campaign,
        "segments": [],
    }

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
                "Restart HIL clean before running single-session stack."
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
            f"Single-session hover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} yaw={math.degrees(yaw0):.1f}deg",
            flush=True,
        )

        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        t_pre = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < t_pre:
            send_position_target_local_ned(mav, target_x, target_y, baseline_z, yaw0)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        t0 = time.time()
        ramp_end = t0 + args.ramp_seconds
        while time.time() < ramp_end:
            now = time.time()
            alpha = min(1.0, max(0.0, (now - t0) / max(0.1, args.ramp_seconds)))
            z_sp = baseline_z + alpha * (target_z - baseline_z)
            send_position_target_local_ned(mav, target_x, target_y, z_sp, yaw0)
            mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.setpoint_period)

        latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe = observe_hold_phase(
            mav,
            label="Direct hover",
            duration_s=args.direct_settle_seconds,
            report_period=args.report_period,
            ref_x=target_x,
            ref_y=target_y,
            ref_z=target_z,
            send_direct_setpoint=True,
            yaw=yaw0,
        )
        ensure_stable_hold(
            label="Direct hover",
            max_xy=max_xy,
            max_z_err=max_z_err,
            max_tilt=max_tilt,
            failsafe=failsafe,
            xy_limit=args.direct_xy_limit,
            z_tolerance=args.direct_z_tolerance,
            tilt_limit_deg=args.direct_tilt_limit_deg,
        )

        set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", float(latest_lpos.x), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", float(latest_lpos.y), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", float(latest_lpos.z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", float(latest_att.yaw), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("custom_pos_control enabled for single-session stack", flush=True)

        latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe = observe_hold_phase(
            mav,
            label="Initial custom hold",
            duration_s=args.custom_hold_seconds,
            report_period=args.report_period,
            ref_x=float(latest_lpos.x),
            ref_y=float(latest_lpos.y),
            ref_z=float(latest_lpos.z),
        )
        ensure_stable_hold(
            label="Initial custom hold",
            max_xy=max_xy,
            max_z_err=max_z_err,
            max_tilt=max_tilt,
            failsafe=failsafe,
            xy_limit=args.custom_xy_limit,
            z_tolerance=args.custom_z_tolerance,
            tilt_limit_deg=args.custom_tilt_limit_deg,
        )

        for profile in CAMPAIGNS[args.campaign]["ident_profiles"]:
            hold_x = float(latest_lpos.x)
            hold_y = float(latest_lpos.y)
            hold_z = float(latest_lpos.z)
            hold_yaw = float(latest_att.yaw)
            print(f"Starting ident {profile} from x={hold_x:.3f} y={hold_y:.3f} z={hold_z:.3f}", flush=True)
            set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_POS_X", hold_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(mav, "TRJ_POS_Y", hold_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(mav, "TRJ_POS_Z", hold_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(mav, "TRJ_POS_YAW", hold_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(mav, "CST_POS_CTRL_TYP", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_IDENT_PROF", IDENTIFICATION_PROFILES[profile], mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_MODE_CMD", 2, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe = observe_ident(
                mav,
                duration_s=identification_duration_s(profile) + args.ident_tail_seconds,
                profile=profile,
                anchor_x=hold_x,
                anchor_y=hold_y,
                anchor_z=hold_z,
                ref_z=hold_z,
                report_period=args.report_period,
            )
            summary["segments"].append(
                {
                    "name": f"ident_{profile}",
                    "kind": "ident",
                    "max_xy_m": max_xy,
                    "max_z_err_m": max_z_err,
                    "max_tilt_deg": max_tilt,
                    "failsafe": failsafe,
                }
            )
            best_effort_return_to_custom_hold(
                mav,
                hold_x=hold_x,
                hold_y=hold_y,
                hold_z=hold_z,
                hold_yaw=hold_yaw,
            )
            latest_lpos, latest_att = settle_custom_hold(
                mav,
                ref_x=hold_x,
                ref_y=hold_y,
                ref_z=hold_z,
                duration_s=args.between_segments_hold_seconds,
                report_period=args.report_period,
            )

        for traj_id in CAMPAIGNS[args.campaign]["trajectory_ids"]:
            hold_x = float(latest_lpos.x)
            hold_y = float(latest_lpos.y)
            hold_z = float(latest_lpos.z)
            hold_yaw = float(latest_att.yaw)
            print(f"Starting trajectory {traj_id} from x={hold_x:.3f} y={hold_y:.3f} z={hold_z:.3f}", flush=True)
            set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_anchor_from_position(mav, hold_x, hold_y, hold_z)
            set_param(mav, "TRJ_ACTIVE_ID", traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_MODE_CMD", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            latest_lpos, latest_att, max_xy, max_z_err, max_tilt, failsafe = observe_builtin_trajectory(
                mav,
                traj_id=traj_id,
                duration_s=TRAJ_DURATION_S[traj_id] + args.trajectory_tail_seconds,
                anchor_x=hold_x,
                anchor_y=hold_y,
                anchor_z=hold_z,
                report_period=args.report_period,
            )
            summary["segments"].append(
                {
                    "name": f"traj_{traj_id}",
                    "kind": "trajectory",
                    "max_xy_m": max_xy,
                    "max_z_err_m": max_z_err,
                    "max_tilt_deg": max_tilt,
                    "failsafe": failsafe,
                }
            )
            best_effort_return_to_custom_hold(
                mav,
                hold_x=hold_x,
                hold_y=hold_y,
                hold_z=hold_z,
                hold_yaw=hold_yaw,
            )
            latest_lpos, latest_att = settle_custom_hold(
                mav,
                ref_x=hold_x,
                ref_y=hold_y,
                ref_z=hold_z,
                duration_s=args.between_segments_hold_seconds,
                report_period=args.report_period,
            )

        if output_json:
            output_json.parent.mkdir(parents=True, exist_ok=True)
            output_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
        return 0
    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
