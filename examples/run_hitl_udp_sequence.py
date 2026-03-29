#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from experimental_validation.hitl_catalog import (
    identification_duration_s,
    identification_profile_index,
    trajectory_duration_s,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one HITL identification profile or trajectory using only the jMAVSim UDP link."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--kind", choices=["ident", "trajectory"], required=True)
    parser.add_argument("--name", help="Identification profile name.")
    parser.add_argument("--traj-id", type=int, help="Trajectory id.")
    parser.add_argument("--hover-z", type=float, default=-3.0, help="Expected hover z in local NED.")
    parser.add_argument("--hover-timeout", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=6.0)
    parser.add_argument("--tail-seconds", type=float, default=3.0)
    return parser.parse_args()


def wait_heartbeat(mav, timeout: float = 20.0) -> None:
    heartbeat = mav.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise TimeoutError("No UDP heartbeat received from jMAVSim/PX4")


def wait_command_ack(mav, command: int, timeout: float = 5.0) -> int:
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return int(msg.result)
    raise TimeoutError(f"No COMMAND_ACK for {command}")


def set_param(mav, name: str, value: float, param_type: int, timeout: float = 8.0) -> None:
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode("ascii"),
        float(value),
        param_type,
    )
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        param_id = msg.param_id.decode(errors="ignore").rstrip("\x00")
        if param_id != name:
            continue
        if math.isclose(float(msg.param_value), float(value), rel_tol=0.0, abs_tol=1e-3):
            return
    raise TimeoutError(f"PARAM_VALUE confirmation missing for {name}={value}")


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
    result = wait_command_ack(mav, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Arm rejected with ACK result {result}")


def set_offboard(mav) -> None:
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
    result = wait_command_ack(mav, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"OFFBOARD rejected with ACK result {result}")


def wait_for_hover(mav, target_z: float, timeout: float) -> None:
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if not msg:
            continue
        z = float(msg.z)
        print(f"LOCAL_POSITION_NED z={z:.3f}", flush=True)
        if abs(z - target_z) <= 0.5:
            return
    raise TimeoutError(f"Vehicle did not reach hover band around z={target_z}")


def main() -> int:
    args = parse_args()
    mav = mavutil.mavlink_connection(args.endpoint, autoreconnect=False)
    wait_heartbeat(mav)

    set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

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

    arm(mav)
    set_offboard(mav)
    wait_for_hover(mav, args.hover_z, args.hover_timeout)
    time.sleep(args.settle_seconds)

    print(f"Starting {trigger_name}", flush=True)
    set_param(mav, "TRJ_MODE_CMD", trigger_value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(run_duration + args.tail_seconds)
    set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    print(f"Completed {trigger_name}", flush=True)
    mav.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
