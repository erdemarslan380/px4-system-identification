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
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_local_position,
    wait_for_sim_ready,
    wait_heartbeat,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baseline HIL: MAVLink manual takeoff in Position mode, then OFFBOARD pose hold."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--target-z", type=float, default=-2.0)
    parser.add_argument("--climb-timeout", type=float, default=25.0)
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    parser.add_argument("--stable-window", type=float, default=2.0)
    parser.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--setpoint-period", type=float, default=0.05)
    parser.add_argument("--manual-period", type=float, default=0.05)
    parser.add_argument("--hover-z-tolerance", type=float, default=0.35)
    parser.add_argument("--offboard-entry-z-margin", type=float, default=0.35)
    parser.add_argument("--xy-limit", type=float, default=0.25)
    parser.add_argument("--z-tolerance", type=float, default=0.25)
    parser.add_argument("--tilt-limit-deg", type=float, default=8.0)
    parser.add_argument("--stable-tilt-before-offboard-deg", type=float, default=5.0)
    parser.add_argument("--stable-horiz-speed", type=float, default=0.18)
    parser.add_argument("--stable-vert-speed", type=float, default=0.12)
    parser.add_argument("--arm-attempts", type=int, default=3)
    return parser.parse_args()


def set_position_mode(mav) -> None:
    custom_mode = 3 << 16
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        29,
        3,
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


class ManualControlThread:
    def __init__(self, mav, period_s: float) -> None:
        self._mav = mav
        self._period_s = period_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._x = 0
        self._y = 0
        self._z = 0
        self._r = 0
        self._thread = threading.Thread(target=self._run, name="manual-control", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def update(self, *, x: int | None = None, y: int | None = None, z: int | None = None, r: int | None = None) -> None:
        with self._lock:
            if x is not None:
                self._x = int(max(-1000, min(1000, x)))
            if y is not None:
                self._y = int(max(-1000, min(1000, y)))
            if z is not None:
                self._z = int(max(0, min(1000, z)))
            if r is not None:
                self._r = int(max(-1000, min(1000, r)))

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                x = self._x
                y = self._y
                z = self._z
                r = self._r
            self._mav.mav.manual_control_send(
                self._mav.target_system,
                x,
                y,
                z,
                r,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self._stop.wait(self._period_s)


def wait_for_attitude(mav, timeout: float = 3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        return msg
    raise TimeoutError("No ATTITUDE received")


def wait_for_manual_hover(
    mav,
    *,
    target_z: float,
    min_entry_z: float,
    timeout: float,
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
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            latest_lpos = msg
        elif msg.get_type() == "ATTITUDE":
            latest_att = msg

        now = time.time()
        if latest_lpos and latest_att and now >= next_report:
            print(
                f"Manual climb x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if not latest_lpos or not latest_att:
            continue

        current_z = float(latest_lpos.z)
        horiz_speed = math.hypot(float(latest_lpos.vx), float(latest_lpos.vy))
        vert_speed = abs(float(latest_lpos.vz))
        tilt_deg = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

        stable = (
            current_z <= min_entry_z
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

    raise TimeoutError(
        f"Vehicle did not settle into manual hover near z={target_z:.3f}; "
        f"last_lpos={latest_lpos} last_att={latest_att}"
    )


def throttle_for_target(current_z: float, target_z: float) -> int:
    remaining = current_z - target_z
    command = 500.0 + remaining * 120.0
    if remaining > 0.8:
        command = max(command, 660.0)
    elif remaining > 0.3:
        command = max(command, 560.0)
    return int(max(450.0, min(720.0, command)))


def best_effort_disarm(mav) -> None:
    try:
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
    except Exception:
        pass


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    manual = ManualControlThread(mav, args.manual_period)
    manual.start()

    try:
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_P", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "FD_FAIL_R", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        att0 = wait_for_attitude(mav)
        yaw0 = float(att0.yaw)
        target_z = baseline_z + args.target_z

        print(
            f"MAVLink manual takeoff target x={baseline_x:.3f} y={baseline_y:.3f} z={target_z:.3f}",
            flush=True,
        )

        set_position_mode(mav)
        manual.update(z=0)
        arm_with_retries(mav, attempts=max(1, args.arm_attempts))

        climb_deadline = time.time() + args.climb_timeout
        next_report = 0.0
        latest_lpos = None
        latest_att = None

        while time.time() < climb_deadline:
            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.manual_period)
            if not msg:
                manual.update(z=700)
                continue

            if msg.get_type() == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue

            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_lpos = msg
                throttle_cmd = throttle_for_target(float(msg.z), target_z)
                manual.update(z=throttle_cmd)
            elif msg.get_type() == "ATTITUDE":
                latest_att = msg

            now = time.time()
            if latest_lpos and latest_att and now >= next_report:
                print(
                    f"Position takeoff x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                    f"thr={throttle_for_target(float(latest_lpos.z), target_z)} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

            if latest_lpos and float(latest_lpos.z) <= target_z + args.hover_z_tolerance:
                break

        manual.update(z=500)

        hold_msg, hold_att = wait_for_manual_hover(
            mav,
            target_z=target_z,
            min_entry_z=target_z + args.offboard_entry_z_margin,
            timeout=max(5.0, args.climb_timeout),
            max_horiz_speed=args.stable_horiz_speed,
            max_vert_speed=args.stable_vert_speed,
            max_tilt_deg=args.stable_tilt_before_offboard_deg,
            stable_window=args.stable_window,
            report_period=args.report_period,
        )

        hold_x = float(hold_msg.x)
        hold_y = float(hold_msg.y)
        hold_z = float(hold_msg.z)
        hold_yaw = float(hold_att.yaw)

        print(
            f"Switching to OFFBOARD hold at x={hold_x:.3f} y={hold_y:.3f} z={hold_z:.3f}",
            flush=True,
        )

        pre_deadline = time.time() + max(0.5, args.pre_offboard_seconds)
        while time.time() < pre_deadline:
            manual.update(z=500)
            send_position_target_local_ned(mav, hold_x, hold_y, hold_z, hold_yaw)
            time.sleep(args.setpoint_period)

        set_offboard(mav)
        print("OFFBOARD accepted", flush=True)

        latest_lpos = hold_msg
        latest_att = hold_att
        max_xy = 0.0
        max_tilt = 0.0
        success_since = None
        next_report = 0.0
        end = time.time() + args.hold_seconds

        while time.time() < end:
            manual.update(z=500)
            send_position_target_local_ned(mav, hold_x, hold_y, hold_z, hold_yaw)
            msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=args.setpoint_period)
            if not msg:
                continue

            if msg.get_type() == "STATUSTEXT":
                print(decode_statustext(msg), flush=True)
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_lpos = msg
                max_xy = max(max_xy, math.hypot(float(msg.x - hold_x), float(msg.y - hold_y)))
            elif msg.get_type() == "ATTITUDE":
                latest_att = msg
                max_tilt = max(max_tilt, math.degrees(max(abs(float(msg.roll)), abs(float(msg.pitch)))))

            now = time.time()
            if latest_lpos and latest_att and now >= next_report:
                print(
                    f"Offboard hold x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                    f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                    f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f}",
                    flush=True,
                )
                next_report = now + args.report_period

            if latest_lpos and latest_att:
                xy_err = math.hypot(float(latest_lpos.x - hold_x), float(latest_lpos.y - hold_y))
                z_err = abs(float(latest_lpos.z) - hold_z)
                tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

                if xy_err <= args.xy_limit and z_err <= args.z_tolerance and tilt <= args.tilt_limit_deg:
                    if success_since is None:
                        success_since = now
                else:
                    success_since = None

        print(f"Manual takeoff -> OFFBOARD hold summary: max_xy={max_xy:.3f}m max_tilt={max_tilt:.2f}deg", flush=True)
        return 0

    finally:
        best_effort_disarm(mav)
        manual.stop()
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.0)
        mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
