#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import math
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.hitl_catalog import (
    identification_duration_s,
    identification_profile_index,
    trajectory_duration_s,
)


_ORIGINAL_ADD_MESSAGE = mavutil.add_message


def install_pymavlink_message_store_patch() -> None:
    if getattr(mavutil, "_px4_sysid_add_message_patched", False):
        return

    def _patched_add_message(messages, mtype, msg):
        try:
            return _ORIGINAL_ADD_MESSAGE(messages, mtype, msg)
        except TypeError as exc:
            if "'NoneType' object does not support item assignment" not in str(exc):
                raise

            if getattr(msg, "_instance_field", None) is None:
                raise

            instance_value = getattr(msg, msg._instance_field, None)
            if instance_value is None:
                raise

            current = messages.get(mtype)
            if current is None:
                current = copy.copy(msg)
                messages[mtype] = current

            if getattr(current, "_instances", None) is None:
                current._instances = {}

            return _ORIGINAL_ADD_MESSAGE(messages, mtype, msg)

    mavutil.add_message = _patched_add_message
    mavutil._px4_sysid_add_message_patched = True


install_pymavlink_message_store_patch()


def normalize_param_id(param_id: object) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode(errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def decode_param_value(raw_value: float, param_type: int) -> float | int:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        packed = struct.pack("<f", float(raw_value))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<b", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<B", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<h", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<H", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<i", packed)[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<I", packed)[0]
    return float(raw_value)


def encode_param_value(value: float | int, param_type: int) -> float:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        int_value = int(round(float(value)))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<f", struct.pack("<bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<f", struct.pack("<Bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<f", struct.pack("<hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<f", struct.pack("<Hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<f", struct.pack("<i", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<f", struct.pack("<I", int_value))[0]
    return float(value)


def param_value_matches(raw_value: float, expected: float, param_type: int) -> bool:
    if math.isclose(float(raw_value), float(expected), rel_tol=0.0, abs_tol=1e-3):
        return True
    decoded = decode_param_value(raw_value, param_type)
    if isinstance(decoded, int):
        return decoded == int(round(expected))
    return math.isclose(float(decoded), float(expected), rel_tol=0.0, abs_tol=1e-3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one HITL identification profile or trajectory over a MAVLink endpoint."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud if --endpoint is a device path.")
    parser.add_argument("--kind", choices=["ident", "trajectory"], required=True)
    parser.add_argument("--name", help="Identification profile name.")
    parser.add_argument("--traj-id", type=int, help="Trajectory id.")
    parser.add_argument(
        "--hover-z",
        type=float,
        default=-3.0,
        help="Desired relative z step from the current LOCAL_POSITION_NED z at OFFBOARD entry.",
    )
    parser.add_argument("--hover-timeout", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=6.0)
    parser.add_argument(
        "--manual-control-mode",
        type=int,
        choices=range(0, 9),
        default=1,
        help=(
            "Value written to COM_RC_IN_MODE before arming. "
            "Default 1 keeps MAVLink manual control alive for HIL takeoff while "
            "disabling RC-specific checks. "
            "Use 0 to keep a physical RC receiver active during HIL mode-switch tests."
        ),
    )
    parser.add_argument(
        "--pre-offboard-seconds",
        type=float,
        default=3.0,
        help="Short dwell after arming and before OFFBOARD. Useful for HIL startup settling.",
    )
    parser.add_argument(
        "--arm-attempts",
        type=int,
        default=3,
        help="How many times to retry arming before giving up.",
    )
    parser.add_argument(
        "--sim-ready-timeout",
        type=float,
        default=20.0,
        help="How long to wait for ATTITUDE and, when required, stable LOCAL_POSITION_NED before arming.",
    )
    parser.add_argument(
        "--sim-ready-min-local-samples",
        type=int,
        default=3,
        help="Number of finite LOCAL_POSITION_NED samples required to declare the estimator ready.",
    )
    parser.add_argument(
        "--allow-missing-local-position",
        action="store_true",
        help="For HIL links that do not stream LOCAL_POSITION_NED, use a timed hover settle and keep the existing anchor params.",
    )
    parser.add_argument(
        "--blind-hover-seconds",
        type=float,
        default=12.0,
        help="Timed settle used with --allow-missing-local-position when LOCAL_POSITION_NED is unavailable.",
    )
    parser.add_argument("--tail-seconds", type=float, default=3.0)
    parser.add_argument(
        "--post-hold-seconds",
        type=float,
        default=5.0,
        help="Seconds to hold in Position mode after trajectory/ident before landing.",
    )
    parser.add_argument(
        "--auto-land",
        action="store_true",
        help="Command NAV_LAND after the post-hold window.",
    )
    parser.add_argument(
        "--strict-reset",
        action="store_true",
        help="Fail if resetting TRJ_MODE_CMD back to position mode is not confirmed at the end of the run.",
    )
    return parser.parse_args()


def try_set_param(mav, name: str, value, param_type: int) -> None:
    try:
        set_param(mav, name, value, param_type)
    except Exception as exc:
        print(f"Best-effort param set skipped: {name}={value} ({exc})", flush=True)


def wait_heartbeat(mav, timeout: float = 20.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        send_gcs_heartbeat(mav)
        heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None:
            return
    raise TimeoutError("No MAVLink heartbeat received from jMAVSim/PX4")


def send_gcs_heartbeat(mav) -> None:
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


class ManualControlKeepalive:
    def __init__(self, mav, period_s: float = 0.05) -> None:
        self._mav = mav
        self._period_s = period_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="manual-control-keepalive", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            # Neutral MAVLink manual-control source for HIL startup and takeoff.
            self._mav.mav.manual_control_send(
                self._mav.target_system,
                0,
                0,
                500,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self._stop.wait(self._period_s)


def send_nsh_command(
    mav,
    command: str,
    *,
    timeout: float = 6.0,
    quiet_timeout: float = 0.7,
) -> str:
    payload = [ord(ch) for ch in (command + "\n")]
    if len(payload) > 70:
        raise ValueError(f"Command too long for SERIAL_CONTROL: {command}")
    payload.extend([0] * (70 - len(payload)))

    mav.mav.serial_control_send(
        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
        0,
        0,
        len(command) + 1,
        payload,
    )

    end_time = time.time() + timeout
    quiet_deadline = None
    chunks: list[str] = []

    while time.time() < end_time:
        message = mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if message is not None and message.count:
            text = "".join(chr(x) for x in message.data[: message.count])
            chunks.append(text)
            quiet_deadline = time.time() + quiet_timeout
            if "nsh>" in text:
                break
        elif quiet_deadline is not None and time.time() >= quiet_deadline:
            break

    return "".join(chunks)


def best_effort_nsh_sequence(mav, commands: list[str]) -> None:
    for cmd in commands:
        try:
            send_nsh_command(mav, cmd)
        except Exception as exc:
            print(f"NSH command skipped: {cmd} ({exc})", flush=True)


def decode_statustext(msg) -> str:
    text = getattr(msg, "text", "")
    if isinstance(text, bytes):
        return text.decode(errors="ignore").rstrip("\x00")
    return str(text).rstrip("\x00")


def start_gcs_heartbeat_thread(mav, period_s: float = 1.0) -> tuple[threading.Event, threading.Thread]:
    stop_event = threading.Event()

    def _worker() -> None:
        while not stop_event.is_set():
            send_gcs_heartbeat(mav)
            stop_event.wait(period_s)

    thread = threading.Thread(target=_worker, name="gcs-heartbeat", daemon=True)
    thread.start()
    return stop_event, thread


def wait_command_ack(mav, command: int, timeout: float = 5.0) -> int:
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return int(msg.result)
    raise TimeoutError(f"No COMMAND_ACK for {command}")


def heartbeat_is_armed(msg) -> bool:
    return bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def heartbeat_main_mode(msg) -> int:
    return (int(getattr(msg, "custom_mode", 0)) >> 16) & 0xFF


def heartbeat_is_offboard(msg) -> bool:
    return heartbeat_main_mode(msg) == 6


def wait_for_arm_confirmation(mav, timeout: float = 8.0) -> None:
    deadline = time.time() + timeout
    ack_result = None
    ack_accepted = False

    while time.time() < deadline:
        msg = mav.recv_match(type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            ack_result = int(msg.result)
            if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                ack_accepted = True

        elif msg_type == "HEARTBEAT" and heartbeat_is_armed(msg):
            return

        elif msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)

    if ack_result is not None and ack_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Arm rejected with ACK result {ack_result}")

    if ack_accepted:
        raise TimeoutError("Arm ACK was accepted but HEARTBEAT never reported ARMED")

    raise TimeoutError("Arm command was not confirmed by ACK or ARMED HEARTBEAT")


def wait_for_offboard_confirmation(mav, timeout: float = 8.0) -> None:
    deadline = time.time() + timeout
    ack_result = None

    while time.time() < deadline:
        msg = mav.recv_match(type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            ack_result = int(msg.result)
            if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return

        elif msg_type == "HEARTBEAT" and heartbeat_is_offboard(msg):
            return

        elif msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)

    if ack_result is not None and ack_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"OFFBOARD rejected with ACK result {ack_result}")
    raise TimeoutError("OFFBOARD command was not confirmed by ACK or HEARTBEAT")


def request_message_interval(
    mav,
    message_id: int,
    interval_us: int = 100_000,
    timeout: float = 3.0,
) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )

    try:
        result = wait_command_ack(mav, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, timeout=timeout)
    except TimeoutError:
        return

    if result not in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    ):
        raise RuntimeError(
            f"MAV_CMD_SET_MESSAGE_INTERVAL for message {message_id} rejected with ACK result {result}"
        )


def set_param(mav, name: str, value: float, param_type: int, timeout: float = 8.0) -> None:
    encoded_value = encode_param_value(value, param_type)
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode("ascii"),
        float(encoded_value),
        param_type,
    )
    end = time.time() + timeout
    while time.time() < end:
        mav.mav.param_request_read_send(
            mav.target_system,
            mav.target_component,
            name.encode("ascii"),
            -1,
        )
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        param_id = normalize_param_id(msg.param_id)
        if param_id != name:
            continue
        if param_value_matches(float(msg.param_value), float(value), param_type):
            return
    raise TimeoutError(f"PARAM_VALUE confirmation missing for {name}={value}")


def read_param(mav, name: str, param_type: int, timeout: float = 8.0) -> float | int:
    end = time.time() + timeout
    while time.time() < end:
        mav.mav.param_request_read_send(
            mav.target_system,
            mav.target_component,
            name.encode("ascii"),
            -1,
        )
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        param_id = normalize_param_id(msg.param_id)
        if param_id != name:
            continue
        return decode_param_value(float(msg.param_value), param_type)
    raise TimeoutError(f"Failed to read parameter {name}")


def arm(mav) -> None:
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    wait_for_arm_confirmation(mav)


def arm_with_retries(mav, attempts: int) -> None:
    last_error: Exception | None = None
    for attempt in range(1, attempts + 1):
        try:
            arm(mav)
            return
        except Exception as exc:  # pragma: no cover - exercised with live vehicle only
            last_error = exc
            print(f"Arm attempt {attempt}/{attempts} failed: {exc}", flush=True)
            time.sleep(2.0)

    if last_error is not None:
        raise last_error


def wait_for_command_ack(
    mav,
    command: int,
    *,
    timeout: float = 8.0,
    accepted_results: tuple[int, ...] = (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    ),
) -> int:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue

        if int(getattr(msg, "command", -1)) != int(command):
            continue

        result = int(getattr(msg, "result", mavutil.mavlink.MAV_RESULT_FAILED))
        if result not in accepted_results:
            raise RuntimeError(f"Command {command} was rejected with ACK result {result}")
        return result

    raise TimeoutError(f"Command {command} was not acknowledged within {timeout:.1f} s")


def send_nav_takeoff(mav) -> None:
    nan = float("nan")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        nan,
        nan,
        nan,
        nan,
        nan,
        nan,
        nan,
    )
    wait_for_command_ack(mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)


def set_offboard(mav) -> None:
    custom_mode = 6 << 16
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
        6,
        0,
        0,
        0,
        0,
        0,
    )
    wait_for_offboard_confirmation(mav)


def set_position_mode(mav) -> None:
    custom_mode = 3 << 16
    mav.mav.set_mode_send(
        mav.target_system,
        mav.target_component,
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


def send_nav_land(mav) -> None:
    nan = float("nan")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        nan,
        nan,
        nan,
        nan,
        nan,
        nan,
        nan,
    )
    wait_for_command_ack(mav, mavutil.mavlink.MAV_CMD_NAV_LAND)


def wait_for_local_position(mav, timeout: float = 5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg:
            return msg
    raise TimeoutError("No LOCAL_POSITION_NED received")


def local_position_sample_is_reasonable(msg) -> bool:
    values = (float(msg.x), float(msg.y), float(msg.z))
    # Before takeoff in HIL we expect the estimator to settle near the ground origin.
    # Reject large startup spikes so we don't lock an obviously bad hover baseline.
    return (
        all(math.isfinite(value) for value in values)
        and abs(float(msg.x)) < 5.0
        and abs(float(msg.y)) < 5.0
        and abs(float(msg.z)) < 5.0
    )


def collect_reasonable_local_position_samples(mav, duration_s: float = 3.0):
    deadline = time.time() + duration_s
    samples = []

    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=0.5)

        if not msg:
            continue

        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue

        if local_position_sample_is_reasonable(msg):
            samples.append((float(msg.x), float(msg.y), float(msg.z)))

    return samples


def robust_ground_baseline(mav) -> tuple[float, float, float]:
    samples = collect_reasonable_local_position_samples(mav, duration_s=3.0)

    if len(samples) < 5:
        return 0.0, 0.0, 0.0

    xs = sorted(sample[0] for sample in samples)
    ys = sorted(sample[1] for sample in samples)
    zs = sorted(sample[2] for sample in samples)
    mid = len(samples) // 2
    return xs[mid], ys[mid], zs[mid]


def wait_for_ground_quiet(
    mav,
    *,
    duration_s: float = 2.0,
    timeout: float = 10.0,
    pos_window_xy: float = 0.25,
    pos_window_z: float = 0.25,
    speed_window: float = 0.10,
    tilt_limit_deg: float = 5.0,
):
    baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
    deadline = time.time() + timeout
    quiet_since = None
    latest_local_position = None
    latest_attitude = None

    while time.time() < deadline:
        msg = mav.recv_match(
            type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"],
            blocking=True,
            timeout=0.5,
        )
        if not msg:
            quiet_since = None
            continue

        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        if mtype == "LOCAL_POSITION_NED":
            latest_local_position = msg
        elif mtype == "ATTITUDE":
            latest_attitude = msg

        if latest_local_position is None or latest_attitude is None:
            quiet_since = None
            continue

        lp = latest_local_position
        att = latest_attitude
        rel_x = float(lp.x) - baseline_x
        rel_y = float(lp.y) - baseline_y
        rel_z = float(lp.z) - baseline_z
        quiet = (
            local_position_sample_is_reasonable(lp)
            and math.hypot(rel_x, rel_y) <= pos_window_xy
            and abs(rel_z) <= pos_window_z
            and math.hypot(float(lp.vx), float(lp.vy)) <= speed_window
            and abs(float(lp.vz)) <= speed_window
            and math.degrees(max(abs(float(att.roll)), abs(float(att.pitch)))) <= tilt_limit_deg
        )

        if quiet:
            quiet_since = quiet_since or time.time()
            if (time.time() - quiet_since) >= duration_s:
                return latest_local_position, latest_attitude
        else:
            quiet_since = None

    raise TimeoutError("Simulation never settled into a quiet ground state")


def set_position_target_absolute(mav, x: float, y: float, z: float, yaw: float = 0.0) -> None:
    set_param(mav, "TRJ_POS_ABS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(mav, "TRJ_POS_X", x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_Y", y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_Z", z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_YAW", yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def set_position_target_relative(mav, x: float, y: float, z: float, yaw: float = 0.0) -> None:
    set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(mav, "TRJ_POS_X", x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_Y", y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_Z", z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_POS_YAW", yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def hover_target_from_local_position(msg, hover_z: float) -> tuple[float, float, float]:
    return float(msg.x), float(msg.y), float(msg.z) + hover_z


def wait_for_sim_ready(
    mav,
    *,
    timeout: float,
    require_local_position: bool,
    min_local_samples: int,
):
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)

    deadline = time.time() + timeout
    attitude_seen = False
    local_position_seen = 0
    latest_local_position = None

    while time.time() < deadline:
        msg = mav.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED", "STATUSTEXT"],
            blocking=True,
            timeout=1.0,
        )
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            attitude_seen = True

        elif msg_type == "LOCAL_POSITION_NED":
            if local_position_sample_is_reasonable(msg):
                local_position_seen += 1
                latest_local_position = msg
            else:
                local_position_seen = 0

        elif msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)

        if attitude_seen and (not require_local_position or local_position_seen >= min_local_samples):
            return latest_local_position

    missing = []
    if not attitude_seen:
        missing.append("ATTITUDE")
    if require_local_position and local_position_seen < min_local_samples:
        missing.append("stable LOCAL_POSITION_NED")
    raise TimeoutError("Simulation not ready: missing " + ", ".join(missing))


def wait_for_hover(mav, target_delta_z: float, timeout: float):
    first_msg = wait_for_local_position(mav, timeout=min(timeout, 5.0))
    baseline_z = float(first_msg.z)
    target_z = baseline_z + target_delta_z
    print(f"Hover baseline z={baseline_z:.3f}, target z={target_z:.3f}", flush=True)
    end = time.time() + timeout
    last_msg = first_msg
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if not msg:
            continue
        z = float(msg.z)
        print(f"LOCAL_POSITION_NED z={z:.3f}", flush=True)
        last_msg = msg
        if abs(z - target_z) <= 0.5:
            return msg
    raise TimeoutError(f"Vehicle did not reach hover band around z={target_z}")


def wait_for_hover_target_z(mav, target_z: float, timeout: float):
    print(f"Waiting for hover target z={target_z:.3f}", flush=True)
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if not msg:
            continue
        z = float(msg.z)
        print(f"LOCAL_POSITION_NED z={z:.3f}", flush=True)
        if abs(z - target_z) <= 0.5:
            return msg
    raise TimeoutError(f"Vehicle did not reach hover band around z={target_z}")


def set_anchor_from_position(mav, x: float, y: float, z: float) -> None:
    set_param(mav, "TRJ_ANCHOR_X", x, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_ANCHOR_Y", y, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_ANCHOR_Z", z, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def reset_mode_cmd(mav, strict: bool) -> bool:
    try:
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        return True
    except TimeoutError as exc:
        if strict:
            raise
        print(f"Warning: final TRJ_MODE_CMD reset was not confirmed: {exc}", flush=True)
        return False


def prepare_hover_and_anchor(
    mav,
    *,
    hover_z: float,
    hover_timeout: float,
    settle_seconds: float,
    arm_attempts: int,
    manual_control_mode: int,
    pre_offboard_seconds: float,
    sim_ready_timeout: float,
    sim_ready_min_local_samples: int,
    allow_missing_local_position: bool,
    blind_hover_seconds: float,
) -> None:
    sim_ready_msg = wait_for_sim_ready(
        mav,
        timeout=sim_ready_timeout,
        require_local_position=not allow_missing_local_position,
        min_local_samples=max(1, sim_ready_min_local_samples),
    )

    set_param(mav, "COM_RC_IN_MODE", manual_control_mode, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(mav, "MIS_TAKEOFF_ALT", max(0.5, abs(float(hover_z))), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)

    send_nav_takeoff(mav)
    arm_with_retries(mav, attempts=max(1, arm_attempts))

    try:
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        if baseline_x == baseline_y == baseline_z == 0.0 and sim_ready_msg is not None and local_position_sample_is_reasonable(sim_ready_msg):
            baseline_x = float(sim_ready_msg.x)
            baseline_y = float(sim_ready_msg.y)
            baseline_z = float(sim_ready_msg.z)

        target_x, target_y, target_z = baseline_x, baseline_y, baseline_z + hover_z
        print(
            f"Ground baseline x={baseline_x:.3f}, y={baseline_y:.3f}, z={baseline_z:.3f} -> hover target z={target_z:.3f}",
            flush=True,
        )
        wait_for_hover_target_z(mav, target_z, hover_timeout)
        time.sleep(settle_seconds)
        anchor_msg = wait_for_local_position(mav, timeout=3.0)
    except TimeoutError:
        if not allow_missing_local_position:
            raise
        print(
            f"LOCAL_POSITION_NED not available; using blind hover settle for {blind_hover_seconds:.1f} s and the existing TRJ_ANCHOR_* params",
            flush=True,
        )
        time.sleep(blind_hover_seconds)
        if pre_offboard_seconds > 0.0:
            time.sleep(pre_offboard_seconds)
        set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
        set_offboard(mav)
        return

    set_anchor_from_position(mav, float(target_x), float(target_y), float(anchor_msg.z))
    set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
    print(
        f"Anchor set from takeoff hover: x={float(target_x):.3f}, "
        f"y={float(target_y):.3f}, z={float(anchor_msg.z):.3f}",
        flush=True,
    )
    if pre_offboard_seconds > 0.0:
        time.sleep(pre_offboard_seconds)
    set_offboard(mav)


def wait_run_window(
    mav,
    duration_s: float,
    *,
    keepalive_offboard: bool = False,
    keepalive_period: float = 0.2,
) -> None:
    deadline = time.time() + duration_s
    next_keepalive = 0.0
    while time.time() < deadline:
        now = time.time()
        if keepalive_offboard and now >= next_keepalive:
            set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
            next_keepalive = now + keepalive_period
        timeout = max(0.0, min(1.0, deadline - time.time()))
        msg = mav.recv_match(blocking=True, timeout=timeout)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)


def offboard_keepalive(mav, duration_s: float, period_s: float = 0.2) -> None:
    end = time.time() + max(0.0, duration_s)
    while time.time() < end:
        set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
        time.sleep(period_s)


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud
    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)
    manual_keepalive = ManualControlKeepalive(mav)
    manual_keepalive.start()

    try:
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_RC_LOSS_T", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_FAIL_ACT_T", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CBRK_FLIGHTTERM", 121212, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_IO_SAFETY", 22027, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "CBRK_USB_CHK", 197848, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTMODE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "RC_MAP_FLTM_BTN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_ARM_WO_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        # Ignore RC/GCS loss during the fixed takeoff -> offboard flow.
        # Bit 1 = auto modes, bit 2 = offboard, so 2 + 4 = 6.
        try_set_param(mav, "COM_RCL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_DLL_EXCEPT", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_set_param(mav, "COM_OF_LOSS_T", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        try_set_param(mav, "COM_OBL_RC_ACT", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

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

        if args.kind == "trajectory":
            if args.traj_id is None:
                raise SystemExit("--traj-id is required for --kind trajectory")
            set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_ACTIVE_ID", args.traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            run_duration = trajectory_duration_s(args.traj_id)
            trigger_name = f"trajectory {args.traj_id}"
            trigger_value = 1
        else:
            if not args.name:
                raise SystemExit("--name is required for --kind ident")
            set_param(mav, "CST_POS_CTRL_TYP", 6, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(
                mav,
                "TRJ_IDENT_PROF",
                identification_profile_index(args.name),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            run_duration = identification_duration_s(args.name)
            trigger_name = f"identification {args.name}"
            trigger_value = 2

        prepare_hover_and_anchor(
            mav,
            hover_z=args.hover_z,
            hover_timeout=args.hover_timeout,
            settle_seconds=args.settle_seconds,
            arm_attempts=args.arm_attempts,
            manual_control_mode=args.manual_control_mode,
            pre_offboard_seconds=args.pre_offboard_seconds,
            sim_ready_timeout=args.sim_ready_timeout,
            sim_ready_min_local_samples=args.sim_ready_min_local_samples,
            allow_missing_local_position=args.allow_missing_local_position,
            blind_hover_seconds=args.blind_hover_seconds,
        )

        # Keep Offboard alive briefly before triggering the trajectory.
        offboard_keepalive(mav, 2.0, period_s=0.2)

        print(f"Starting {trigger_name}", flush=True)
        set_param(mav, "TRJ_MODE_CMD", trigger_value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        wait_run_window(mav, run_duration + args.tail_seconds, keepalive_offboard=True)
        reset_mode_cmd(mav, strict=args.strict_reset)
        if args.post_hold_seconds > 0.0:
            set_position_mode(mav)
            set_position_target_relative(mav, 0.0, 0.0, 0.0, 0.0)
            print(f"Post-hold {args.post_hold_seconds:.1f}s in Position mode", flush=True)
            wait_run_window(mav, args.post_hold_seconds)
        if args.auto_land:
            print("Commanding NAV_LAND", flush=True)
            send_nav_land(mav)
            wait_run_window(mav, max(5.0, args.tail_seconds + 2.0))
        print(f"Completed {trigger_name}", flush=True)
    finally:
        manual_keepalive.stop()
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.5)
        mav.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
