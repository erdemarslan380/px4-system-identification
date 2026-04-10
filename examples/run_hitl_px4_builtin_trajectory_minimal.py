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
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.trajectory_catalog import validation_trajectory_id_map, load_validation_trajectory_samples

from run_hitl_udp_sequence import (
    arm_with_retries,
    best_effort_nsh_sequence,
    decode_statustext,
    request_message_interval,
    robust_ground_baseline,
    send_nav_takeoff,
    set_anchor_from_position,
    set_offboard,
    set_param,
    set_position_mode,
    start_gcs_heartbeat_thread,
    wait_for_ground_quiet,
    wait_for_local_position,
    wait_for_sim_ready,
    wait_heartbeat,
)
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    disarm,
    execute_staged_hover_entry,
    observe_armed_ground_hold,
    observe_hold_phase,
    send_position_target_local_ned,
)


TRAJ_DURATION_S = {
    100: 27.7,  # hairpin
    101: 23.18,  # lemniscate
    102: 18.9,  # circle
    103: 11.0,  # time_optimal_30s
    104: 23.44,  # minimum_snap_50s
}

_TRAJ_SAMPLE_CACHE: dict[int, object] = {}


def _validation_samples(traj_id: int):
    samples = _TRAJ_SAMPLE_CACHE.get(traj_id)
    if samples is None:
        entry = validation_trajectory_id_map()[traj_id]
        samples = load_validation_trajectory_samples(entry)
        _TRAJ_SAMPLE_CACHE[traj_id] = samples
    return samples


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Minimal baseline -> custom_pos hold -> built-in trajectory_reader trajectory 100..104."
    )
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--traj-id", type=int, choices=sorted(TRAJ_DURATION_S), default=102)
    p.add_argument("--hover-z", type=float, default=-3.0)
    p.add_argument("--ramp-seconds", type=float, default=8.0)
    p.add_argument("--nav-takeoff-alt", type=float, default=None)
    p.add_argument("--takeoff-timeout", type=float, default=45.0)
    p.add_argument("--takeoff-settle-seconds", type=float, default=0.0)
    p.add_argument("--takeoff-stable-window", type=float, default=0.5)
    p.add_argument("--takeoff-z-tolerance", type=float, default=0.25)
    p.add_argument("--takeoff-max-horiz-speed", type=float, default=0.15)
    p.add_argument("--takeoff-max-vert-speed", type=float, default=0.60)
    p.add_argument("--takeoff-tilt-limit-deg", type=float, default=5.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--direct-settle-seconds", type=float, default=2.0)
    p.add_argument("--custom-handover-seconds", type=float, default=2.0)
    p.add_argument("--custom-hold-seconds", type=float, default=15.0)
    p.add_argument("--trajectory-tail-seconds", type=float, default=10.0)
    p.add_argument("--setpoint-period", type=float, default=0.05)
    p.add_argument("--report-period", type=float, default=0.5)
    p.add_argument("--arm-attempts", type=int, default=3)
    p.add_argument("--direct-xy-limit", type=float, default=0.20)
    p.add_argument("--direct-z-tolerance", type=float, default=0.50)
    p.add_argument("--direct-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--custom-xy-limit", type=float, default=0.20)
    p.add_argument("--custom-z-tolerance", type=float, default=0.20)
    p.add_argument("--custom-tilt-limit-deg", type=float, default=6.0)
    p.add_argument("--trajectory-xy-envelop-limit", type=float, default=3.50)
    p.add_argument("--trajectory-z-tolerance", type=float, default=0.50)
    p.add_argument("--trajectory-tilt-limit-deg", type=float, default=20.0)
    p.add_argument("--ground-xy-window", type=float, default=0.25)
    p.add_argument("--ground-z-window", type=float, default=0.25)
    return p.parse_args()


def try_set_param(mav, name: str, value, param_type: int) -> None:
    try:
        set_param(mav, name, value, param_type)
    except Exception as exc:
        print(f"Best-effort param set skipped: {name}={value} ({exc})", flush=True)


def observe_builtin_trajectory(
    mav,
    *,
    traj_id: int,
    duration_s: float,
    anchor_x: float,
    anchor_y: float,
    anchor_z: float,
    report_period: float,
) -> tuple[object, object, float, float, float, bool]:
    end = time.time() + duration_s
    start_time = time.time()
    latest_lpos = None
    latest_att = None
    next_report = 0.0
    max_xy_err = 0.0
    max_z_err = 0.0
    max_tilt = 0.0
    failsafe_seen = False
    last_telemetry_time = None
    telemetry_timeout_s = 1.0
    ref_samples = _validation_samples(traj_id)
    ref_first = ref_samples[0, :3].astype(float)
    ref_offset = (anchor_x - ref_first[0], anchor_y - ref_first[1], anchor_z - ref_first[2])

    while time.time() < end:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
        if not msg:
            if latest_lpos is not None and latest_att is not None and last_telemetry_time is not None:
                if time.time() - last_telemetry_time > telemetry_timeout_s:
                    raise RuntimeError(
                        f"Telemetry went stale during built-in trajectory phase (> {telemetry_timeout_s:.1f} s without "
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
            elapsed_s = max(0.0, time.time() - start_time)
            sample_index = min(int(elapsed_s / 0.02), ref_samples.shape[0] - 1)
            ref_x = float(ref_samples[sample_index, 0]) + ref_offset[0]
            ref_y = float(ref_samples[sample_index, 1]) + ref_offset[1]
            ref_z = float(ref_samples[sample_index, 2]) + ref_offset[2]
            max_xy_err = max(max_xy_err, math.hypot(float(msg.x - ref_x), float(msg.y - ref_y)))
            max_z_err = max(max_z_err, abs(float(msg.z - ref_z)))
            last_telemetry_time = time.time()
        elif mtype == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))
            last_telemetry_time = time.time()
        now = time.time()
        if now >= next_report and latest_lpos and latest_att:
            print(
                f"Built-in trajectory x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("No LOCAL_POSITION_NED/ATTITUDE data during built-in trajectory phase")
    if last_telemetry_time is None or time.time() - last_telemetry_time > telemetry_timeout_s:
        raise RuntimeError("Telemetry went stale before built-in trajectory phase finished")
    return latest_lpos, latest_att, max_xy_err, max_z_err, max_tilt, failsafe_seen


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


def observe_passive_ground_hold(
    mav,
    *,
    duration_s: float,
    report_period: float,
):
    end = time.time() + duration_s
    latest_lpos = None
    latest_att = None
    next_report = 0.0

    while time.time() < end:
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
                f"Armed ground hold x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

    return latest_lpos, latest_att


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


def main() -> int:
    args = parse_args()
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

        # No physical RC is attached in this HIL workflow. Keep PX4 from
        # treating missing manual-control input as a flight-time failsafe.
        try_set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_RC_LOSS_T", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_FAIL_ACT_T", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OF_LOSS_T", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OBL_RC_ACT", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DL_LOSS_T", 30, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "NAV_RCL_ACT", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_RCL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DLL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "GF_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_IO_SAFETY", 22027, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_USB_CHK", 197848, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTMODE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTM_BTN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        best_effort_nsh_sequence(
            mav,
            [
                "sdmount",
                "mkdir -p /fs/microsd/tracking_logs",
                "mkdir -p /fs/microsd/identification_logs",
                "trajectory_reader start",
                "custom_pos_control start",
                "custom_pos_control set px4_default",
            ],
        )

        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # In HIL, commanding a much higher NAV_TAKEOFF altitude than the
        # intended OFFBOARD entry altitude can trigger a pre-offboard failsafe
        # during the climb/recovery segment. Keep takeoff aligned with the
        # target hover altitude unless the caller explicitly overrides it.
        nav_takeoff_alt = float(args.nav_takeoff_alt) if args.nav_takeoff_alt is not None else max(0.5, abs(float(args.hover_z)))
        set_param(mav, "MIS_TAKEOFF_ALT", nav_takeoff_alt, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        print("Waiting for sim ready", flush=True)
        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        print("Sim ready", flush=True)
        print("Waiting for quiet ground state", flush=True)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        print("Ground state quiet", flush=True)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > args.ground_xy_window or abs(baseline_z) > args.ground_z_window:
            print(
                f"Ground baseline offset warning: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}",
                flush=True,
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
            f"Built-in trajectory hover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} "
            f"yaw={math.degrees(yaw0):.1f}deg traj_id={args.traj_id} nav_takeoff_alt={nav_takeoff_alt:.2f}",
            flush=True,
        )

        print("Arming", flush=True)
        arm_with_retries(mav, attempts=max(1, args.arm_attempts))
        print("Arm confirmed", flush=True)

        print("Observing armed ground hold", flush=True)
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
        print("Armed ground hold complete", flush=True)

        print("Requesting OFFBOARD", flush=True)
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

        try_set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

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

        latest_lpos, latest_att, traj_max_xy_err, traj_max_z_err, traj_max_tilt, traj_failsafe = observe_builtin_trajectory(
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
            f"Built-in trajectory summary: traj_id={args.traj_id} max_xy_err={traj_max_xy_err:.3f}m "
            f"max_z_err={traj_max_z_err:.3f}m max_tilt={traj_max_tilt:.2f}deg "
            f"final_xy={final_xy:.3f}m final_z_err={final_z_err:.3f}m final_tilt={final_tilt:.2f}deg "
            f"failsafe={traj_failsafe}",
            flush=True,
        )

        if (
            traj_failsafe
            or traj_max_z_err > args.trajectory_z_tolerance
            or traj_max_tilt > args.trajectory_tilt_limit_deg
        ):
            raise RuntimeError(
                f"Built-in trajectory unstable: path_xy_extent={traj_path_xy_extent:.3f}m max_z_err={traj_max_z_err:.3f}m "
                f"max_tilt={traj_max_tilt:.2f}deg failsafe={traj_failsafe}"
            )

        print("trajectory_reader built-in trajectory stable", flush=True)
        successful_completion = True
        return 0
    finally:
        if not successful_completion:
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
