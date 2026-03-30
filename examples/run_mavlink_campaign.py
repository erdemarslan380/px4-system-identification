#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.hitl_catalog import campaign_expected_duration_s
from examples.run_hitl_udp_sequence import (
    arm,
    read_param,
    reset_mode_cmd,
    set_offboard,
    set_param,
    start_gcs_heartbeat_thread,
    wait_for_hover,
    wait_for_local_position,
    wait_heartbeat,
)

CAMPAIGN_PARAM_VALUES = {
    "identification_only": 1,
    "full_stack": 2,
    "trajectory_only": 3,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start the built-in trajectory_reader campaign over MAVLink and wait for completion."
    )
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud if --endpoint is a device path.")
    parser.add_argument("--campaign", choices=sorted(CAMPAIGN_PARAM_VALUES), default="full_stack")
    parser.add_argument(
        "--prepare-hover",
        action="store_true",
        help="Arm, switch to OFFBOARD and wait for the hover band before starting the campaign.",
    )
    parser.add_argument(
        "--hover-z",
        type=float,
        default=-3.0,
        help="Desired relative z step from the current LOCAL_POSITION_NED z when --prepare-hover is used.",
    )
    parser.add_argument("--hover-timeout", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument(
        "--allow-missing-local-position",
        action="store_true",
        help="For HIL links that do not stream LOCAL_POSITION_NED, fall back to a timed hover settle instead of failing.",
    )
    parser.add_argument(
        "--blind-hover-seconds",
        type=float,
        default=12.0,
        help="Timed settle used with --allow-missing-local-position when LOCAL_POSITION_NED is unavailable.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        help="Overall timeout. Defaults to the expected campaign duration plus a safety margin.",
    )
    parser.add_argument(
        "--strict-stop",
        action="store_true",
        help="Fail if stopping the campaign back to position mode is not confirmed.",
    )
    parser.add_argument("--heartbeat-warmup", type=float, default=2.0)
    parser.add_argument("--arm-attempts", type=int, default=3)
    parser.add_argument(
        "--manual-control-mode",
        type=int,
        choices=range(0, 9),
        default=4,
        help=(
            "Value written to COM_RC_IN_MODE during --prepare-hover. "
            "Default 4 disables manual control so sticks cannot interfere. "
            "Use 0 to keep a physical RC receiver active during HIL mode-switch tests."
        ),
    )
    return parser.parse_args()


def default_timeout_s(campaign_name: str) -> float:
    return campaign_expected_duration_s(campaign_name) + 90.0


def decode_statustext(msg) -> str:
    text = getattr(msg, "text", "")
    if isinstance(text, bytes):
        return text.decode(errors="ignore").rstrip("\x00")
    return str(text).rstrip("\x00")


def wait_for_campaign_completion(mav, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    last_position_report = 0.0

    while time.time() < deadline:
        try:
            campaign_status = read_param(
                mav,
                "TRJ_CAMPAIGN_STA",
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                timeout=0.8,
            )
            if int(campaign_status) == 2:
                return
            if int(campaign_status) == 3:
                raise RuntimeError("Campaign aborted")
        except TimeoutError:
            pass

        msg = mav.recv_match(blocking=True, timeout=1.0)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            if "Campaign completed" in text:
                return
            if "Campaign aborted" in text or "Campaign failed" in text:
                raise RuntimeError(text)

        elif msg_type == "LOCAL_POSITION_NED":
            now = time.time()
            if now - last_position_report >= 10.0:
                print(
                    f"LOCAL_POSITION_NED x={float(msg.x):.2f} y={float(msg.y):.2f} z={float(msg.z):.2f}",
                    flush=True,
                )
                last_position_report = now

    raise TimeoutError(f"Campaign did not complete within {timeout_s:.1f} s")


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


def prepare_hover(mav, args: argparse.Namespace) -> None:
    set_param(mav, "COM_RC_IN_MODE", args.manual_control_mode, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    arm_with_retries(mav, attempts=max(1, args.arm_attempts))
    set_offboard(mav)

    try:
        wait_for_hover(mav, args.hover_z, args.hover_timeout)
        time.sleep(args.settle_seconds)
    except TimeoutError:
        if not args.allow_missing_local_position:
            raise
        print(
            f"LOCAL_POSITION_NED not available; using blind hover settle for {args.blind_hover_seconds:.1f} s",
            flush=True,
        )
        time.sleep(args.blind_hover_seconds)


def main() -> int:
    args = parse_args()
    connection_kwargs = {"autoreconnect": False}
    if args.endpoint.startswith("/dev/"):
        connection_kwargs["baud"] = args.baud

    mav = mavutil.mavlink_connection(args.endpoint, **connection_kwargs)
    wait_heartbeat(mav)
    stop_heartbeats, heartbeat_thread = start_gcs_heartbeat_thread(mav)

    try:
        time.sleep(args.heartbeat_warmup)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_CAMPAIGN", CAMPAIGN_PARAM_VALUES[args.campaign], mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        if args.prepare_hover:
            prepare_hover(mav, args)
        else:
            try:
                wait_for_local_position(mav, timeout=5.0)
            except TimeoutError:
                if not args.allow_missing_local_position:
                    raise
                print("LOCAL_POSITION_NED not available; starting campaign without a position gate", flush=True)

        print(f"Starting campaign {args.campaign}", flush=True)
        set_param(mav, "TRJ_CAMPAIGN_CMD", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        wait_for_campaign_completion(mav, timeout_s=args.timeout or default_timeout_s(args.campaign))
        reset_mode_cmd(mav, strict=args.strict_stop)
        print(f"Completed campaign {args.campaign}", flush=True)
    finally:
        stop_heartbeats.set()
        heartbeat_thread.join(timeout=1.5)
        mav.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
