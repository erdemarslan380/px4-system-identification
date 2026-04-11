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

from experimental_validation.hitl_catalog import campaign_expected_duration_s, campaign_ident_profiles
from run_hitl_px4_builtin_ident_minimal import (
    best_effort_disarm,
    best_effort_set_param,
    IDENTIFICATION_PROFILES,
    set_land_mode,
    set_position_mode,
    wait_until_landed,
)
from run_hitl_udp_sequence import (
    arm_with_retries,
    decode_statustext,
    encode_param_value,
    read_param,
    request_message_interval,
    robust_ground_baseline,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    execute_staged_hover_entry,
    observe_armed_ground_hold,
    observe_hold_phase,
    send_position_target_local_ned,
)

PROFILE_NAME_BY_INDEX = {int(value): name for name, value in IDENTIFICATION_PROFILES.items()}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run all built-in identification profiles sequentially in one stable HIL flight.")
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--profiles", nargs="*", default=campaign_ident_profiles("identification_only"))
    p.add_argument("--hover-z", type=float, default=-5.0)
    p.add_argument("--ramp-seconds", type=float, default=8.0)
    p.add_argument("--direct-settle-seconds", type=float, default=2.0)
    p.add_argument("--custom-handover-seconds", type=float, default=2.0)
    p.add_argument("--custom-hold-seconds", type=float, default=4.0)
    p.add_argument("--recovery-settle-seconds", type=float, default=1.5)
    p.add_argument("--between-segments-hold-seconds", type=float, default=2.0)
    p.add_argument("--ident-tail-seconds", type=float, default=4.0)
    p.add_argument("--report-period", type=float, default=0.5)
    p.add_argument("--setpoint-period", type=float, default=0.05)
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
    p.add_argument("--output-json", default="")
    return p.parse_args()


def _stabilize_profile_defaults(args: argparse.Namespace, profile: str) -> None:
    if profile != "yaw_sweep":
        return
    args.direct_xy_limit = max(args.direct_xy_limit, 1.50)
    args.direct_z_tolerance = max(args.direct_z_tolerance, 0.80)
    args.direct_tilt_limit_deg = max(args.direct_tilt_limit_deg, 35.0)
    args.direct_settle_seconds = max(args.direct_settle_seconds, 4.0)
    args.custom_xy_limit = max(args.custom_xy_limit, 0.35)
    args.custom_z_tolerance = max(args.custom_z_tolerance, 0.10)
    args.custom_tilt_limit_deg = max(args.custom_tilt_limit_deg, 20.0)


def _trigger_identification_campaign(mav, timeout_s: float = 10.0) -> None:
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        b"TRJ_CAMPAIGN_CMD",
        float(encode_param_value(1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            status = int(read_param(mav, "TRJ_CAMPAIGN_STA", mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout=0.6))
            if status != 0:
                return
        except TimeoutError:
            pass

        msg = mav.recv_match(type=["STATUSTEXT", "PARAM_VALUE"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Campaign" in text:
                return
        elif msg.get_type() == "PARAM_VALUE":
            param_id = getattr(msg, "param_id", b"")
            if isinstance(param_id, bytes):
                param_id = param_id.decode(errors="ignore").rstrip("\x00")
            else:
                param_id = str(param_id).rstrip("\x00")
            if param_id == "TRJ_CAMPAIGN_CMD":
                return

    raise TimeoutError("Identification campaign trigger was not acknowledged")


def _monitor_identification_campaign(
    mav,
    *,
    expected_profiles: list[str],
    timeout_s: float,
) -> list[dict[str, object]]:
    deadline = time.monotonic() + timeout_s
    profiles: list[dict[str, object]] = []
    active_profile: str | None = None
    active_start_wall_ns = 0
    active_start_mono_ns = 0
    last_poll = 0.0
    last_position_report = 0.0

    def _record_profile_window(profile_name: str, start_wall_ns: int, end_wall_ns: int, start_mono_ns: int, end_mono_ns: int) -> None:
        if profiles and profiles[-1]["profile"] == profile_name:
            profiles[-1]["host_wall_time_end_ns"] = end_wall_ns
            profiles[-1]["host_mono_time_end_ns"] = end_mono_ns
            return
        profiles.append(
            {
                "profile": profile_name,
                "host_wall_time_start_ns": start_wall_ns,
                "host_wall_time_end_ns": end_wall_ns,
                "host_mono_time_start_ns": start_mono_ns,
                "host_mono_time_end_ns": end_mono_ns,
            }
        )

    while time.monotonic() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
        now_mono = time.monotonic()
        if msg:
            msg_type = msg.get_type()
            if msg_type == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
            elif msg_type == "LOCAL_POSITION_NED" and (time.time() - last_position_report) >= 5.0:
                print(
                    f"Campaign LOCAL_POSITION_NED x={float(msg.x):.2f} y={float(msg.y):.2f} z={float(msg.z):.2f}",
                    flush=True,
                )
                last_position_report = time.time()

        if now_mono - last_poll < 0.5:
            continue
        last_poll = now_mono

        try:
            campaign_status = int(read_param(mav, "TRJ_CAMPAIGN_STA", mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout=0.5))
        except TimeoutError:
            campaign_status = -1

        try:
            mode_cmd = int(read_param(mav, "TRJ_MODE_CMD", mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout=0.5))
        except TimeoutError:
            mode_cmd = -1

        try:
            ident_profile_idx = int(read_param(mav, "TRJ_IDENT_PROF", mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout=0.5))
        except TimeoutError:
            ident_profile_idx = -1

        current_profile = PROFILE_NAME_BY_INDEX.get(ident_profile_idx)
        in_ident = mode_cmd == 2 and current_profile in expected_profiles

        if in_ident and current_profile != active_profile:
            if active_profile is not None:
                _record_profile_window(
                    active_profile,
                    active_start_wall_ns,
                    time.time_ns(),
                    active_start_mono_ns,
                    time.monotonic_ns(),
                )
            active_profile = current_profile
            active_start_wall_ns = time.time_ns()
            active_start_mono_ns = time.monotonic_ns()
            print(f"Campaign identification profile {active_profile} entered", flush=True)

        elif not in_ident and active_profile is not None:
            _record_profile_window(
                active_profile,
                active_start_wall_ns,
                time.time_ns(),
                active_start_mono_ns,
                time.monotonic_ns(),
            )
            print(f"Campaign identification profile {active_profile} completed", flush=True)
            active_profile = None

        if campaign_status == 3:
            raise RuntimeError("Identification campaign aborted")

        if campaign_status == 2 and active_profile is None:
            ordered_profiles = [entry["profile"] for entry in profiles]
            if ordered_profiles != expected_profiles:
                raise RuntimeError(
                    f"Campaign completed with unexpected profile order: expected {expected_profiles}, got {ordered_profiles}"
                )
            return profiles

    raise TimeoutError(f"Identification campaign did not complete within {timeout_s:.1f} s")


def main() -> int:
    args = parse_args()
    kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        kwargs["baud"] = args.baud

    output_json = Path(args.output_json).expanduser().resolve() if args.output_json else None
    mav = mavutil.mavlink_connection(args.endpoint, **kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    successful_completion = False
    baseline_z = 0.0
    summary: dict[str, object] = {"profiles": [], "successful_completion": False}

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
        try:
            wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        except TimeoutError as exc:
            print(f"Warning: ground quiet gate skipped ({exc})", flush=True)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart HIL clean before running built-in identification sequence."
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
            f"Built-in identification sequence hover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} "
            f"yaw={math.degrees(yaw0):.1f}deg",
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
        latest_lpos, latest_att, *_ = execute_staged_hover_entry(
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

        hold_ref_x = float(latest_lpos.x)
        hold_ref_y = float(latest_lpos.y)
        hold_ref_z = float(latest_lpos.z)
        hold_ref_yaw = float(latest_att.yaw)

        set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", hold_ref_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", hold_ref_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", hold_ref_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", hold_ref_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_ANCHOR_X", hold_ref_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_ANCHOR_Y", hold_ref_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_ANCHOR_Z", hold_ref_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("custom_pos_control enabled", flush=True)

        handover_until = time.monotonic() + max(0.0, args.custom_handover_seconds)
        while time.monotonic() < handover_until:
            send_position_target_local_ned(mav, hold_ref_x, hold_ref_y, hold_ref_z, hold_ref_yaw)
            time.sleep(args.setpoint_period)

        latest_lpos, latest_att, custom_max_xy, custom_max_z_err, custom_max_tilt, custom_failsafe = observe_hold_phase(
            mav,
            label="Initial custom hold",
            duration_s=args.custom_hold_seconds,
            report_period=args.report_period,
            ref_x=hold_ref_x,
            ref_y=hold_ref_y,
            ref_z=hold_ref_z,
        )
        if (
            custom_failsafe
            or custom_max_xy > args.custom_xy_limit
            or custom_max_z_err > args.custom_z_tolerance
            or custom_max_tilt > args.custom_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Initial custom hold unstable: max_xy={custom_max_xy:.3f}m max_z_err={custom_max_z_err:.3f}m "
                f"max_tilt={custom_max_tilt:.2f}deg failsafe={custom_failsafe}"
            )

        set_param(mav, "TRJ_POS_X", hold_ref_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", hold_ref_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", hold_ref_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", hold_ref_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_CAMPAIGN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("identification campaign prepared", flush=True)
        _trigger_identification_campaign(mav)
        print("identification campaign started", flush=True)
        summary["profiles"] = _monitor_identification_campaign(
            mav,
            expected_profiles=list(args.profiles),
            timeout_s=campaign_expected_duration_s("identification_only") + 90.0,
        )
        print(json.dumps(summary["profiles"], indent=2), flush=True)

        summary["successful_completion"] = True
        if output_json:
            output_json.parent.mkdir(parents=True, exist_ok=True)
            output_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
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
