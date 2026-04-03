#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
import threading
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
    wait_for_sim_ready,
    wait_heartbeat,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baseline HIL: climb with direct OFFBOARD setpoints, then hand position hold to custom modules."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--hover-z", type=float, default=-2.0)
    parser.add_argument("--ramp-seconds", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--settle-timeout", type=float, default=8.0)
    parser.add_argument("--settle-vxy", type=float, default=0.08)
    parser.add_argument("--settle-vz", type=float, default=0.08)
    parser.add_argument("--settle-pos-tol", type=float, default=0.15)
    parser.add_argument("--hold-seconds", type=float, default=8.0)
    parser.add_argument("--handover-seconds", type=float, default=2.0)
    parser.add_argument("--pre-offboard-seconds", type=float, default=3.0)
    parser.add_argument("--setpoint-period", type=float, default=0.05)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--arm-attempts", type=int, default=3)
    parser.add_argument("--direct-xy-limit", type=float, default=0.20)
    parser.add_argument("--direct-tilt-limit-deg", type=float, default=6.0)
    parser.add_argument("--xy-limit", type=float, default=0.20)
    parser.add_argument("--z-tolerance", type=float, default=0.20)
    parser.add_argument("--tilt-limit-deg", type=float, default=6.0)
    parser.add_argument("--sysid-hold-seconds", type=float, default=8.0)
    parser.add_argument("--sysid-xy-limit", type=float, default=0.20)
    parser.add_argument("--sysid-z-tolerance", type=float, default=0.20)
    parser.add_argument("--sysid-tilt-limit-deg", type=float, default=6.0)
    return parser.parse_args()


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


def try_set_param(mav, name: str, value, param_type: int) -> None:
    try:
        set_param(mav, name, value, param_type)
    except Exception as exc:
        print(f"Best-effort param set skipped: {name}={value} ({exc})", flush=True)


def start_manual_control_thread(mav, period_s: float = 0.1) -> tuple[threading.Event, threading.Thread]:
    stop_event = threading.Event()

    def _worker() -> None:
        while not stop_event.is_set():
            try:
                mav.mav.manual_control_send(
                    mav.target_system,
                    0,    # pitch
                    0,    # roll
                    500,  # neutral throttle
                    0,    # yaw
                    0,
                )
            except Exception:
                return
            stop_event.wait(period_s)

    thread = threading.Thread(target=_worker, name="neutral-manual-control", daemon=True)
    thread.start()
    return stop_event, thread


def observe_hold_phase(
    mav,
    *,
    label: str,
    hold_seconds: float,
    setpoint_period: float,
    report_period: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
) -> tuple[object, object, float, float]:
    end = time.time() + hold_seconds
    max_xy = 0.0
    max_tilt = 0.0
    next_report = 0.0
    latest_lpos = None
    latest_att = None
    failsafe_seen = False

    while time.time() < end:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=setpoint_period)
        if not msg:
            continue

        if msg.get_type() == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Failsafe activated" in text or "Flight termination active" in text:
                failsafe_seen = True
            continue
        if msg.get_type() == "LOCAL_POSITION_NED":
            latest_lpos = msg
            max_xy = max(max_xy, math.hypot(float(msg.x - ref_x), float(msg.y - ref_y)))
        elif msg.get_type() == "ATTITUDE":
            latest_att = msg
            max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))

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
        raise RuntimeError(f"No LOCAL_POSITION_NED/ATTITUDE data during {label.lower()} phase")

    return latest_lpos, latest_att, max_xy, max_tilt, failsafe_seen


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    stop_manual_control, manual_control_thread = start_manual_control_thread(mav)

    try:
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_FAIL_ACT_T", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OF_LOSS_T", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OBL_RC_ACT", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DL_LOSS_T", 30.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "NAV_RCL_ACT", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_RCL_EXCEPT", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DLL_EXCEPT", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "GF_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)

        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        att0 = None
        for _ in range(10):
            msg = mav.recv_match(type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
            if not msg:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue
            att0 = msg
            break

        yaw0 = float(att0.yaw) if att0 else 0.0
        target_x = baseline_x
        target_y = baseline_y
        target_z = baseline_z + args.hover_z

        print(
            f"Module handover target x={target_x:.3f} y={target_y:.3f} z={target_z:.3f} yaw={math.degrees(yaw0):.1f}deg",
            flush=True,
        )

        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        pre_offboard_deadline = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < pre_offboard_deadline:
            send_position_target_local_ned(mav, target_x, target_y, baseline_z, yaw0)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        latest_lpos = None
        latest_att = None
        next_report = 0.0
        t0 = time.time()
        ramp_end = t0 + args.ramp_seconds
        direct_max_xy = 0.0
        direct_max_tilt = 0.0
        direct_failsafe_seen = False

        while time.time() < ramp_end:
            now = time.time()
            alpha = min(1.0, max(0.0, (now - t0) / max(0.1, args.ramp_seconds)))
            z_sp = baseline_z + alpha * (target_z - baseline_z)
            send_position_target_local_ned(mav, target_x, target_y, z_sp, yaw0)

            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.setpoint_period)
            if not msg:
                continue

            if msg.get_type() == "STATUSTEXT":
                text = decode_statustext(msg)
                print(text, flush=True)
                if "Failsafe activated" in text or "Flight termination active" in text:
                    direct_failsafe_seen = True
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_lpos = msg
                direct_max_xy = max(direct_max_xy, math.hypot(float(msg.x - target_x), float(msg.y - target_y)))
            elif msg.get_type() == "ATTITUDE":
                latest_att = msg
                direct_max_tilt = max(
                    direct_max_tilt,
                    math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))),
                )

            if now >= next_report and latest_lpos and latest_att:
                print(
                    f"Direct hover x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

        if latest_lpos is None:
            raise RuntimeError("Did not receive LOCAL_POSITION_NED during direct hover phase")
        if direct_failsafe_seen or direct_max_xy > args.direct_xy_limit or direct_max_tilt > args.direct_tilt_limit_deg:
            raise RuntimeError(
                f"Direct hover unstable: max_xy={direct_max_xy:.3f}m max_tilt={direct_max_tilt:.2f}deg "
                f"failsafe={direct_failsafe_seen}"
            )

        # Hold the final direct setpoint until position and velocity settle.
        settle_deadline = time.time() + max(args.settle_seconds, 0.0) + max(args.settle_timeout, 0.0)
        settle_since = None
        next_report = 0.0
        while time.time() < settle_deadline:
            send_position_target_local_ned(mav, target_x, target_y, target_z, yaw0)
            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.setpoint_period)
            if not msg:
                continue

            if msg.get_type() == "STATUSTEXT":
                text = decode_statustext(msg)
                print(text, flush=True)
                if "Failsafe activated" in text or "Flight termination active" in text:
                    direct_failsafe_seen = True
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_lpos = msg
                direct_max_xy = max(direct_max_xy, math.hypot(float(msg.x - target_x), float(msg.y - target_y)))
                speed_xy = math.hypot(float(msg.vx), float(msg.vy))
                speed_z = abs(float(msg.vz))
                pos_err = math.sqrt(
                    (float(msg.x) - target_x) ** 2
                    + (float(msg.y) - target_y) ** 2
                    + (float(msg.z) - target_z) ** 2
                )
                if (
                    speed_xy <= args.settle_vxy
                    and speed_z <= args.settle_vz
                    and pos_err <= args.settle_pos_tol
                ):
                    settle_since = settle_since or time.time()
                else:
                    settle_since = None
            elif msg.get_type() == "ATTITUDE":
                latest_att = msg
                direct_max_tilt = max(
                    direct_max_tilt,
                    math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))),
                )

            now = time.time()
            if now >= next_report and latest_lpos and latest_att:
                print(
                    f"Direct settle x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

            if settle_since and (time.time() - settle_since) >= args.settle_seconds:
                break

        if not settle_since:
            raise RuntimeError("Direct hover did not settle before module handover")

        handover_x = float(latest_lpos.x)
        handover_y = float(latest_lpos.y)
        handover_z = float(latest_lpos.z)
        handover_yaw = float(latest_att.yaw) if latest_att else yaw0

        print(
            f"Enabling module-managed hold at x={handover_x:.3f} y={handover_y:.3f} z={handover_z:.3f}",
            flush=True,
        )

        set_param(mav, "TRJ_POS_X", handover_x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", handover_y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", handover_z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", handover_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        # Keep direct setpoints alive briefly while the custom stack starts publishing.
        handover_deadline = time.time() + max(0.5, args.handover_seconds)
        while time.time() < handover_deadline:
            send_position_target_local_ned(mav, handover_x, handover_y, handover_z, handover_yaw)
            time.sleep(args.setpoint_period)

        latest_lpos, latest_att, max_xy, max_tilt, failsafe_seen = observe_hold_phase(
            mav,
            label="Module hold",
            hold_seconds=args.hold_seconds,
            setpoint_period=args.setpoint_period,
            report_period=args.report_period,
            ref_x=handover_x,
            ref_y=handover_y,
            ref_z=handover_z,
        )

        print(f"Module hold summary: max_xy={max_xy:.3f}m max_tilt={max_tilt:.2f}deg", flush=True)

        final_xy = math.hypot(float(latest_lpos.x - handover_x), float(latest_lpos.y - handover_y))
        final_z_err = abs(float(latest_lpos.z) - handover_z)
        final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

        if (
            failsafe_seen
            or max_xy > args.xy_limit
            or max_tilt > args.tilt_limit_deg
            or final_xy > args.xy_limit
            or final_z_err > args.z_tolerance
            or final_tilt > args.tilt_limit_deg
        ):
            raise RuntimeError(
                f"Module hold unstable: max_xy={max_xy:.3f}m final_xy={final_xy:.3f}m "
                f"z_err={final_z_err:.3f}m max_tilt={max_tilt:.2f}deg final_tilt={final_tilt:.2f}deg "
                f"failsafe={failsafe_seen}"
            )

        if args.sysid_hold_seconds > 0.0:
            print("Switching hold controller to SYSID", flush=True)
            set_param(mav, "CST_POS_CTRL_TYP", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            latest_lpos, latest_att, max_xy, max_tilt, failsafe_seen = observe_hold_phase(
                mav,
                label="SYSID hold",
                hold_seconds=args.sysid_hold_seconds,
                setpoint_period=args.setpoint_period,
                report_period=args.report_period,
                ref_x=handover_x,
                ref_y=handover_y,
                ref_z=handover_z,
            )

            final_xy = math.hypot(float(latest_lpos.x - handover_x), float(latest_lpos.y - handover_y))
            final_z_err = abs(float(latest_lpos.z) - handover_z)
            final_tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

            print(f"SYSID hold summary: max_xy={max_xy:.3f}m max_tilt={max_tilt:.2f}deg", flush=True)

            if (
                failsafe_seen
                or max_xy > args.sysid_xy_limit
                or max_tilt > args.sysid_tilt_limit_deg
                or final_xy > args.sysid_xy_limit
                or final_z_err > args.sysid_z_tolerance
                or final_tilt > args.sysid_tilt_limit_deg
            ):
                raise RuntimeError(
                    f"SYSID hold unstable: max_xy={max_xy:.3f}m final_xy={final_xy:.3f}m "
                    f"z_err={final_z_err:.3f}m max_tilt={max_tilt:.2f}deg final_tilt={final_tilt:.2f}deg "
                    f"failsafe={failsafe_seen}"
                )

        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        disarm(mav)
        print("Module-managed position hold stable", flush=True)
        return 0

    finally:
        try:
            set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        except Exception:
            pass
        try:
            set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        except Exception:
            pass
        stop_manual_control.set()
        manual_control_thread.join(timeout=1.0)
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
