#!/usr/bin/env python3
"""Run one controller trajectory evaluation on an already-running PX4 SITL instance.

Controls PX4 through local SITL wrappers (px4-commander, px4-trajectory_reader...).
Also writes optional live trajectory trace JSON for dashboard updates.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from controller_profiles import (
    ControllerProfile,
    ensure_default_param_file,
    get_controller_profile,
    param_names,
    param_spec_map,
    parse_param_file,
)
from experimental_validation.qgc_params import parse_qgc_parameter_dump

# For long SITL optimization loops we relax brittle preflight checks that
# otherwise start randomly denying arming after many cycles.
SITL_STABILITY_PARAMS = {
    "CBRK_SUPPLY_CHK": 894281,
    "COM_ARM_WO_GPS": 2,
    "SYS_HAS_MAG": 0,
    "COM_DISARM_PRFLT": -1,
    "COM_DISARM_LAND": 2,
    "COM_FAIL_ACT_T": 0,
    "COM_LOW_BAT_ACT": 0,
    "COM_ARM_MAG_STR": 0,
    "COM_ARM_MAG_ANG": 180,
    "EKF2_MAG_CHECK": 0,
    "NAV_DLL_ACT": 0,
}

# Keep SITL runs lightweight. The optimizer reads trajectory tracking CSV files,
# not ULog, so continuous PX4 file logging is unnecessary overhead here.
RUNTIME_SPEED_PARAMS = {
    "SDLOG_MODE": 0,
    "SDLOG_PROFILE": 0,
}

NON_RETRYABLE_ERROR_TOKENS = (
    "takeoff stalled",
    "takeoff altitude not reached",
    "takeoff timeout budget exhausted",
    "trajectory timeout",
    "vehicle did not stabilize at the takeoff hover point",
)


class Px4Ctl:
    def __init__(self, rootfs: Path, instance_id: int = 0):
        self.rootfs = rootfs.resolve()
        self.bin_dir = self.rootfs.parent / "bin"
        self.instance_id = instance_id

        if not (self.bin_dir / "px4-commander").exists():
            raise RuntimeError(f"PX4 wrapper not found: {(self.bin_dir / 'px4-commander')}")

    def run(self, module: str, args: list[str], timeout_s: float = 8.0) -> str:
        exe = self.bin_dir / f"px4-{module}"
        if not exe.exists():
            raise RuntimeError(f"Module wrapper missing: {exe}")

        proc = subprocess.run(
            [str(exe), "--instance", str(self.instance_id), *args],
            cwd=str(self.rootfs),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )

        out = (proc.stdout or "") + (proc.stderr or "")

        if proc.returncode != 0:
            raise RuntimeError(f"Command failed: {exe.name} {' '.join(args)}\n{out}")

        return out

    def command(self, command_line: str, timeout_s: float = 8.0) -> str:
        tokens = shlex.split(command_line)
        if not tokens:
            return ""
        return self.run(tokens[0], tokens[1:], timeout_s=timeout_s)


def parse_status_field(status_text: str, key: str) -> Optional[str]:
    m = re.search(rf"{re.escape(key)}:\s*([^\r\n]+)", status_text)
    return m.group(1).strip() if m else None


def latest_tracking_log(log_dir: Path, traj_id: int, since_ts: float, ident_profile: str = "") -> Optional[Path]:
    patterns = [
        str(log_dir / f"id_{traj_id}_run_*_start_*.csv"),
        str(log_dir / f"t{traj_id}r*.csv"),
    ]
    if ident_profile:
        patterns.extend([
            str(log_dir / f"{ident_profile}_run_*_start_*.csv"),
            str(log_dir / f"{ident_profile}_r*.csv"),
        ])
    candidates = []
    for pattern in patterns:
        candidates.extend(Path(p) for p in glob.glob(pattern))
    unique_candidates = []
    seen = set()
    for candidate in candidates:
        key = str(candidate.resolve())
        if key in seen:
            continue
        seen.add(key)
        unique_candidates.append(candidate)
    candidates = unique_candidates
    candidates = [p for p in candidates if p.stat().st_mtime >= since_ts - 1.0]
    if not candidates:
        return None
    return sorted(candidates, key=lambda p: p.stat().st_mtime)[-1]


def latest_identification_log(log_dir: Path, ident_profile: str, since_ts: float) -> Optional[Path]:
    patterns = [
        str(log_dir / f"{ident_profile}_r*.csv"),
        str(log_dir / f"{ident_profile}_run_*_start_*.csv"),
    ]
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(Path(p) for p in glob.glob(pattern))
    candidates = [p for p in candidates if p.exists() and p.is_file() and p.stat().st_mtime >= since_ts - 1.0]
    if not candidates:
        return None
    return sorted(candidates, key=lambda p: p.stat().st_mtime)[-1]


def tracking_rows(log_path: Path) -> list[dict]:
    rows = []
    with log_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def write_live_trace(live_trace_json: Optional[Path],
                     eval_id: int,
                     tracking_log: Optional[Path],
                     done: bool,
                     message: str = "",
                     worker_id: int = -1,
                     fitness: Optional[float] = None,
                     sample_trace: Optional[dict] = None) -> None:
    if live_trace_json is None:
        return

    payload = {
        "eval_id": eval_id,
        "worker_id": worker_id,
        "done": done,
        "message": message,
        "fitness": fitness,
        "source": "empty",
        "t": [],
        "ref": {"x": [], "y": [], "z": []},
        "act": {"x": [], "y": [], "z": []},
    }

    if tracking_log is not None and tracking_log.exists():
        payload["source"] = "tracking_log"
        t0_us: Optional[int] = None
        for r in tracking_rows(tracking_log):
            try:
                ts_us = int(float(r["timestamp_us"]))
                if t0_us is None:
                    t0_us = ts_us
                payload["t"].append((ts_us - t0_us) * 1e-6)
                payload["ref"]["x"].append(float(r["ref_x"]))
                payload["ref"]["y"].append(float(r["ref_y"]))
                payload["ref"]["z"].append(float(r["ref_z"]))
                payload["act"]["x"].append(float(r["pos_x"]))
                payload["act"]["y"].append(float(r["pos_y"]))
                payload["act"]["z"].append(float(r["pos_z"]))
            except (KeyError, ValueError):
                continue
    elif sample_trace:
        payload["source"] = "sample_trace"
        payload["t"] = list(sample_trace.get("t", []))
        payload["ref"] = {
            "x": list(sample_trace.get("ref", {}).get("x", [])),
            "y": list(sample_trace.get("ref", {}).get("y", [])),
            "z": list(sample_trace.get("ref", {}).get("z", [])),
        }
        payload["act"] = {
            "x": list(sample_trace.get("act", {}).get("x", [])),
            "y": list(sample_trace.get("act", {}).get("y", [])),
            "z": list(sample_trace.get("act", {}).get("z", [])),
        }

    atomic_write_text(live_trace_json, json.dumps(payload, indent=2))


def start_module_safe(px4: Px4Ctl, module: str) -> None:
    try:
        status_out = px4.command(f"{module} status", timeout_s=4.0)
    except RuntimeError as exc:
        msg = str(exc)
        if "not running" in msg.lower():
            status_out = msg
        else:
            raise

    if "not running" in status_out.lower():
        try:
            px4.command(f"{module} start", timeout_s=6.0)
        except RuntimeError as exc:
            # Some modules return non-zero when already running/racing startup.
            if "already running" not in str(exc).lower():
                raise


def get_param_value(px4: Px4Ctl, name: str) -> Optional[float]:
    out = px4.command(f"param show -q {name}", timeout_s=4.0)
    m = re.search(r"([-+]?\d+(?:\.\d+)?)", out)
    return float(m.group(1)) if m else None


def parse_fixed_param_items(items: list[str]) -> Dict[str, float]:
    parsed: Dict[str, float] = {}
    for raw in items:
        if "=" not in raw:
            raise RuntimeError(f"Invalid --fixed-param value: {raw}")
        name, value = raw.split("=", 1)
        parsed[name.strip()] = float(value.strip())
    return parsed


def apply_base_param_dump(px4: Px4Ctl, path: Optional[str]) -> None:
    if not path:
        return

    param_path = Path(path).resolve()
    if not param_path.exists():
        raise RuntimeError(f"Base parameter file not found: {param_path}")

    params = parse_qgc_parameter_dump(param_path)
    if not params:
        raise RuntimeError(f"No parameters parsed from base parameter file: {param_path}")

    for name, target in params.items():
        try:
            current = get_param_value(px4, name)
            if current is None or abs(current - float(target)) > 1e-6:
                px4.command(f"param set {name} {target}", timeout_s=4.0)
        except Exception:
            continue
    time.sleep(0.20)


def apply_named_params(px4: Px4Ctl, params: Dict[str, float]) -> None:
    for name, target in params.items():
        current = get_param_value(px4, name)
        if current is None or abs(current - float(target)) > 1e-6:
            px4.command(f"param set {name} {target}", timeout_s=4.0)
    time.sleep(0.15)


def apply_controller_params(px4: Px4Ctl, params: Dict[str, float], profile: ControllerProfile) -> None:
    # Update only changed parameters to avoid excessive autosave/verify races.
    specs = param_spec_map(profile)
    for name in param_names(profile):
        current = get_param_value(px4, name)
        target = int(round(params[name])) if specs[name].kind == "int" else float(params[name])
        if current is None or abs(current - float(target)) > 1e-6:
            px4.command(f"param set {name} {target}", timeout_s=4.0)
    # Let autosave settle a bit before arming.
    time.sleep(0.15)


def apply_sitl_stability_params(px4: Px4Ctl) -> None:
    for name, target in SITL_STABILITY_PARAMS.items():
        current = get_param_value(px4, name)
        if current is None or abs(current - float(target)) > 1e-6:
            px4.command(f"param set {name} {target}", timeout_s=4.0)
    time.sleep(0.15)


def apply_runtime_speed_params(px4: Px4Ctl) -> None:
    for name, target in RUNTIME_SPEED_PARAMS.items():
        current = get_param_value(px4, name)
        if current is None or abs(current - float(target)) > 1e-6:
            try:
                px4.command(f"param set {name} {target}", timeout_s=4.0)
            except Exception:
                pass
    try:
        px4.command("logger stop", timeout_s=4.0)
    except Exception:
        pass
    time.sleep(0.05)


def apply_takeoff_alt_param(px4: Px4Ctl, takeoff_alt_m: float) -> None:
    target = float(takeoff_alt_m)
    current = get_param_value(px4, "MIS_TAKEOFF_ALT")
    if current is None or abs(current - target) > 1e-6:
        px4.command(f"param set MIS_TAKEOFF_ALT {target}", timeout_s=4.0)
    time.sleep(0.05)


def relaxed_hover_profile(base: dict) -> dict:
    return {
        "timeout_s": max(float(base.get("timeout_s", 10.0)), 18.0),
        "pos_tol_xy_m": max(float(base.get("pos_tol_xy_m", 0.08)), 0.25),
        "pos_tol_z_m": max(float(base.get("pos_tol_z_m", 0.10)), 0.28),
        "vel_tol_mps": max(float(base.get("vel_tol_mps", 0.18)), 0.40),
        "stable_samples": min(int(base.get("stable_samples", 4)), 3),
    }


def activate_px4_default_controller(px4: Px4Ctl) -> None:
    px4.command("custom_pos_control set px4_default")
    px4.command("custom_pos_control enable")
    time.sleep(0.10)


def activate_target_controller(px4: Px4Ctl,
                               profile: ControllerProfile,
                               fixed_params: Dict[str, float],
                               params: Dict[str, float]) -> None:
    px4.command(f"custom_pos_control set {profile.cli_name}")
    px4.command("custom_pos_control enable")
    if profile.name == "pid":
        apply_named_params(px4, {
            "MPC_USE_HTE": 0.0,
            "MPC_ACC_DECOUPLE": 1.0,
        })
    apply_named_params(px4, fixed_params)
    apply_controller_params(px4, params, profile)
    time.sleep(0.10)


def prepare_target_controller(px4: Px4Ctl,
                              profile: ControllerProfile,
                              fixed_params: Dict[str, float],
                              params: Dict[str, float]) -> None:
    if profile.name == "pid":
        apply_named_params(px4, {
            "MPC_USE_HTE": 0.0,
            "MPC_ACC_DECOUPLE": 1.0,
        })
    apply_named_params(px4, fixed_params)
    apply_controller_params(px4, params, profile)
    time.sleep(0.05)


def get_local_z(px4: Px4Ctl) -> float:
    out = px4.command("listener vehicle_local_position 1", timeout_s=4.0)
    m = re.search(r"\bz:\s*([-+]?\d+(?:\.\d+)?)", out)
    if not m:
        raise RuntimeError(f"Cannot parse local z from listener output\n{out}")
    return float(m.group(1))


def get_attitude_thrust_cmd(px4: Px4Ctl) -> Optional[float]:
    out = px4.command("listener vehicle_attitude_setpoint 1", timeout_s=4.0)
    m = re.search(
        r"thrust_body:\s*\[\s*[-+]?\d+(?:\.\d+)?\s*,\s*[-+]?\d+(?:\.\d+)?\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\]",
        out,
    )
    if not m:
        return None
    return float(m.group(1))


def get_local_vz(px4: Px4Ctl) -> float:
    out = px4.command("listener vehicle_local_position 1", timeout_s=4.0)
    m = re.search(r"\bvz:\s*([-+]?\d+(?:\.\d+)?)", out)
    if not m:
        raise RuntimeError(f"Cannot parse local vz from listener output\n{out}")
    return float(m.group(1))


def get_local_position_sample(px4: Px4Ctl) -> dict:
    out = px4.command("listener vehicle_local_position 1", timeout_s=4.0)

    def parse(name: str) -> float:
        m = re.search(rf"\b{name}:\s*([-+]?\d+(?:\.\d+)?)", out)
        if not m:
            raise RuntimeError(f"Cannot parse {name} from listener output\n{out}")
        return float(m.group(1))

    return {
        "x": parse("x"),
        "y": parse("y"),
        "z": parse("z"),
        "vx": parse("vx"),
        "vy": parse("vy"),
        "vz": parse("vz"),
    }


def get_vehicle_status_fields(px4: Px4Ctl) -> dict:
    out = px4.command("listener vehicle_status 1", timeout_s=4.0)
    fields: dict = {}
    m = re.search(r"\barming_state:\s*(\d+)", out)
    if m:
        fields["arming_state"] = int(m.group(1))
    m = re.search(r"\bnav_state:\s*(\d+)", out)
    if m:
        fields["nav_state"] = int(m.group(1))
    m = re.search(r"\bpre_flight_checks_pass:\s*(True|False)", out)
    if m:
        fields["pre_flight_checks_pass"] = (m.group(1) == "True")
    return fields


def get_hover_thrust_status(px4: Px4Ctl) -> dict:
    out = px4.command("listener hover_thrust_estimate 1", timeout_s=4.0)
    m = re.search(r"\bhover_thrust:\s*([-+]?\d+(?:\.\d+)?)", out)
    return {
        "valid": parse_bool_field(out, "valid"),
        "hover_thrust": float(m.group(1)) if m else math.nan,
    }


def parse_bool_field(text: str, key: str) -> Optional[bool]:
    m = re.search(rf"\b{re.escape(key)}:\s*(True|False)", text)
    if not m:
        return None
    return m.group(1) == "True"


def get_local_position_flags(px4: Px4Ctl) -> dict:
    out = px4.command("listener vehicle_local_position 1", timeout_s=4.0)
    flags = {
        "xy_valid": parse_bool_field(out, "xy_valid"),
        "z_valid": parse_bool_field(out, "z_valid"),
        "v_xy_valid": parse_bool_field(out, "v_xy_valid"),
        "v_z_valid": parse_bool_field(out, "v_z_valid"),
        "dead_reckoning": parse_bool_field(out, "dead_reckoning"),
    }
    return flags


def is_landed(px4: Px4Ctl) -> bool:
    out = px4.command("listener vehicle_land_detected 1", timeout_s=4.0)
    m = re.search(r"\blanded:\s*(True|False)", out)
    return bool(m and m.group(1) == "True")


def compute_tracking_rmse(log_path: Path, controller_name: str) -> float:
    sq = []
    with log_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("controller", "").strip().lower() != controller_name:
                continue
            dx = float(row["ref_x"]) - float(row["pos_x"])
            dy = float(row["ref_y"]) - float(row["pos_y"])
            dz = float(row["ref_z"]) - float(row["pos_z"])
            sq.append(dx * dx + dy * dy + dz * dz)

    if not sq:
        raise RuntimeError(f"No {controller_name} rows found in {log_path}")

    return math.sqrt(sum(sq) / len(sq))


def wait_until_altitude(px4: Px4Ctl,
                        target_alt_m: float,
                        timeout_s: float,
                        stall_timeout_s: float = 14.0,
                        progress_eps_m: float = 0.10,
                        completion_tol_m: float = 0.10) -> None:
    t0 = time.time()
    start_z: Optional[float] = None
    best_progress = 0.0
    last_progress_t = t0

    while time.time() - t0 < timeout_s:
        now = time.time()
        z = get_local_z(px4)
        if start_z is None:
            start_z = z

        # NED: altitude gain means z gets more negative.
        progress = max(0.0, (start_z - z))
        if progress > best_progress + 0.02:
            best_progress = progress
            last_progress_t = now

        if z <= -(abs(target_alt_m) - max(0.0, completion_tol_m)):
            return

        if now - last_progress_t > stall_timeout_s and best_progress < max(progress_eps_m, 0.25 * abs(target_alt_m)):
            raise RuntimeError(
                f"Takeoff stalled: progress={best_progress:.2f}m in {now - t0:.1f}s "
                f"(target={abs(target_alt_m):.2f}m)"
            )

        time.sleep(0.25)

    raise RuntimeError(f"Takeoff altitude not reached in {timeout_s}s")


def wait_hover_stable(px4: Px4Ctl,
                      target_alt_m: float,
                      timeout_s: float = 20.0,
                      pos_tol_xy_m: float = 0.02,
                      pos_tol_z_m: float = 0.03,
                      vel_tol_mps: float = 0.08,
                      stable_samples: int = 10) -> None:
    t0 = time.time()
    stable_count = 0

    while time.time() - t0 < timeout_s:
        sample = get_local_position_sample(px4)
        status = get_vehicle_status_fields(px4)

        pos_ok = (
            abs(sample["x"]) <= pos_tol_xy_m
            and abs(sample["y"]) <= pos_tol_xy_m
            and abs(sample["z"] + abs(target_alt_m)) <= pos_tol_z_m
        )
        vel_ok = (
            abs(sample["vx"]) <= vel_tol_mps
            and abs(sample["vy"]) <= vel_tol_mps
            and abs(sample["vz"]) <= vel_tol_mps
        )

        if pos_ok and vel_ok:
            stable_count += 1
            if stable_count >= stable_samples:
                return
        else:
            stable_count = 0

        if (
            status.get("arming_state", -1) != 2
            and is_landed(px4)
            and time.time() - t0 > 1.2
        ):
            raise RuntimeError("Vehicle disarmed before hover stabilization")

        time.sleep(0.2)

    raise RuntimeError("Vehicle did not stabilize at the takeoff hover point")


def hover_stability_profile(controller_name: str, engagement_dwell_s: float | None = None) -> dict:
    return {
        # Common pre-mission hover hold after switching to the selected
        # controller. This is intentionally permissive enough to verify a safe
        # airborne handover, not a perfect hover lock.
        "timeout_s": 14.0,
        "dwell_s": float(engagement_dwell_s) if engagement_dwell_s is not None else 2.0,
        "pos_guard_xy_m": 0.55,
        "pos_guard_z_m": 0.60,
        "vel_guard_mps": 0.85,
    }


def allow_relaxed_hover_gate(controller_name: str) -> bool:
    return True


def initial_hover_profile(controller_name: str, relaxed: bool = False) -> dict:
    gate = {
        # Common pre-switch hover gate used while PX4 default control is still
        # active. This is intentionally controller-independent because the
        # selected controller has not taken over yet.
        "timeout_s": 12.0,
        "pos_tol_xy_m": 0.18,
        "pos_tol_z_m": 0.20,
        "vel_tol_mps": 0.28,
        "stable_samples": 3,
    }

    return relaxed_hover_profile(gate) if relaxed else gate


def relaxed_engagement_profile(base: dict) -> dict:
    return {
        "timeout_s": max(float(base.get("timeout_s", 10.0)), 18.0),
        "dwell_s": max(float(base.get("dwell_s", 1.5)), 1.5),
        "pos_guard_xy_m": max(float(base.get("pos_guard_xy_m", 0.45)), 0.75),
        "pos_guard_z_m": max(float(base.get("pos_guard_z_m", 0.50)), 0.80),
        "vel_guard_mps": max(float(base.get("vel_guard_mps", 0.75)), 1.10),
    }


def wait_controller_engaged(px4: Px4Ctl,
                            target_alt_m: float,
                            timeout_s: float = 12.0,
                            dwell_s: float = 2.0,
                            pos_guard_xy_m: float = 0.55,
                            pos_guard_z_m: float = 0.60,
                            vel_guard_mps: float = 0.85) -> None:
    t0 = time.time()
    guard_since: Optional[float] = None

    while time.time() - t0 < timeout_s:
        sample = get_local_position_sample(px4)
        status = get_vehicle_status_fields(px4)

        xy_err = max(abs(sample["x"]), abs(sample["y"]))
        z_err = abs(sample["z"] + abs(target_alt_m))
        vel_peak = max(abs(sample["vx"]), abs(sample["vy"]), abs(sample["vz"]))

        within_guard = (
            xy_err <= pos_guard_xy_m
            and z_err <= pos_guard_z_m
            and vel_peak <= vel_guard_mps
        )

        if within_guard:
            if guard_since is None:
                guard_since = time.time()
            if time.time() - guard_since >= dwell_s:
                return
        else:
            guard_since = None

        if (
            status.get("arming_state", -1) != 2
            and is_landed(px4)
            and time.time() - t0 > 1.2
        ):
            raise RuntimeError("Vehicle disarmed before controller engagement completed")

        # Gross divergence is treated as an immediate failure instead of
        # waiting for the full timeout budget.
        if (
            xy_err > 1.8 * pos_guard_xy_m
            or z_err > 1.8 * pos_guard_z_m
            or vel_peak > 1.8 * vel_guard_mps
        ):
            raise RuntimeError("Vehicle diverged during controller engagement")

        time.sleep(0.2)

    raise RuntimeError("Vehicle did not remain safely engaged after controller switch")


def should_retry_eval(exc: Exception) -> bool:
    text = str(exc).lower()
    return not any(token in text for token in NON_RETRYABLE_ERROR_TOKENS)


def wait_ground_stable(px4: Px4Ctl,
                       timeout_s: float = 6.0,
                       pos_tol_xy_m: float = 0.12,
                       pos_tol_z_m: float = 0.15,
                       vel_tol_mps: float = 0.15,
                       stable_samples: int = 4) -> None:
    t0 = time.time()
    stable_count = 0

    while time.time() - t0 < timeout_s:
        sample = get_local_position_sample(px4)

        pos_ok = (
            abs(sample["x"]) <= pos_tol_xy_m
            and abs(sample["y"]) <= pos_tol_xy_m
            and abs(sample["z"]) <= pos_tol_z_m
        )
        vel_ok = (
            abs(sample["vx"]) <= vel_tol_mps
            and abs(sample["vy"]) <= vel_tol_mps
            and abs(sample["vz"]) <= vel_tol_mps
        )

        if pos_ok and vel_ok and is_landed(px4):
            stable_count += 1
            if stable_count >= stable_samples:
                return
        else:
            stable_count = 0

        time.sleep(0.2)

    raise RuntimeError("Vehicle did not stabilize on the ground before arming")


def wait_disarmed_and_landed(px4: Px4Ctl, timeout_s: float = 20.0) -> None:
    t0 = time.time()
    stable_cnt = 0
    while time.time() - t0 < timeout_s:
        st = get_vehicle_status_fields(px4)
        arming_state = st.get("arming_state", -1)
        landed = is_landed(px4)
        vz = abs(get_local_vz(px4))
        if arming_state == 1 and landed and vz < 0.2:
            stable_cnt += 1
            if stable_cnt >= 3:
                return
        else:
            stable_cnt = 0
        time.sleep(0.25)
    raise RuntimeError("Vehicle did not reach disarmed+landed stable state")


def wait_preflight_ready(px4: Px4Ctl, timeout_s: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = get_vehicle_status_fields(px4)
        if st.get("pre_flight_checks_pass", False):
            return True
        time.sleep(0.25)
    return False


def wait_armed(px4: Px4Ctl, timeout_s: float = 8.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = get_vehicle_status_fields(px4)
        if st.get("arming_state") == 2:
            return
        time.sleep(0.2)
    raise RuntimeError("Vehicle did not arm")


def wait_hover_thrust_valid(px4: Px4Ctl, timeout_s: float = 8.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hte = get_hover_thrust_status(px4)
        if hte.get("valid") is True and math.isfinite(hte.get("hover_thrust", math.nan)):
            return
        time.sleep(0.2)
    raise RuntimeError("Hover thrust estimate did not become valid")


def wait_navigation_ready(px4: Px4Ctl, timeout_s: float = 20.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        f = get_local_position_flags(px4)
        if (
            f.get("xy_valid") is True
            and f.get("z_valid") is True
            and f.get("v_xy_valid") is True
            and f.get("v_z_valid") is True
            and f.get("dead_reckoning") is False
        ):
            return True
        time.sleep(0.3)
    return False


def bring_vehicle_to_baseline(px4: Px4Ctl) -> None:
    # Keep this sequence explicit to recover from stale TAKEOFF/OFFBOARD intent.
    try:
        px4.command("commander mode auto:loiter")
    except Exception:
        pass
    try:
        px4.command("trajectory_reader set_mode position")
        px4.command("trajectory_reader ref 0 0 -2.0 0")
    except Exception:
        pass
    try:
        px4.command("commander disarm -f")
    except Exception:
        pass
    try:
        wait_disarmed_and_landed(px4, timeout_s=12.0)
    except Exception:
        pass
    try:
        px4.command("commander mode auto:loiter")
    except Exception:
        pass


def takeoff_with_retry(px4: Px4Ctl,
                       target_alt_m: float,
                       timeout_s: float,
                       controller_name: str = "") -> None:
    deadline = time.time() + max(5.0, timeout_s)
    errs: list[str] = []
    controller_name = controller_name.strip().lower()

    def remaining() -> float:
        rem = deadline - time.time()
        if rem <= 1.0:
            raise RuntimeError("Takeoff timeout budget exhausted")
        return rem

    if controller_name in {"pid", "indi"}:
        primary_budget = min(max(14.0, timeout_s * 0.65), max(14.0, timeout_s - 5.0))
        fallback_budget = min(16.0, max(8.0, timeout_s * 0.40))
        primary_stall = min(24.0, max(10.0, timeout_s * 0.45))
        fallback_stall = min(18.0, max(7.0, timeout_s * 0.30))
    else:
        primary_budget = min(max(10.0, timeout_s * 0.55), max(10.0, timeout_s - 8.0))
        fallback_budget = min(10.0, max(4.0, timeout_s * 0.25))
        primary_stall = min(20.0, max(8.0, timeout_s * 0.35))
        fallback_stall = min(14.0, max(5.0, timeout_s * 0.25))

    def settle_ground() -> None:
        try:
            wait_ground_stable(px4, timeout_s=6.0, vel_tol_mps=0.18, stable_samples=3)
        except Exception:
            # Arm -f is still the authoritative check in SITL. This gate is only
            # used to reduce spurious takeoff failures after recent disarm events.
            pass

    # Path A: arm and climb under PX4 default position control in OFFBOARD.
    try:
        try:
            px4.command("commander mode auto:loiter")
        except Exception:
            pass
        settle_ground()
        px4.command("commander arm -f")
        wait_armed(px4, timeout_s=6.0)
        time.sleep(0.55)
        enter_offboard_position_hold(px4, target_alt_m, timeout_s=min(8.0, remaining()))
        wait_until_altitude(
            px4,
            target_alt_m,
            min(primary_budget, remaining()),
            stall_timeout_s=primary_stall,
            progress_eps_m=0.08,
        )
        return
    except Exception as exc:
        errs.append(f"A:{exc}")
        try:
            px4.command("commander disarm -f")
        except Exception:
            pass
        time.sleep(0.6)
        try:
            bring_vehicle_to_baseline(px4)
        except Exception:
            pass

    # Path B: short fallback to plain PX4 takeoff.
    try:
        try:
            px4.command("commander mode auto:loiter")
        except Exception:
            pass
        settle_ground()
        px4.command("commander arm -f")
        wait_armed(px4, timeout_s=4.0)
        time.sleep(0.8)
        px4.command("commander takeoff")
        wait_until_altitude(
            px4,
            target_alt_m,
            min(fallback_budget, remaining()),
            stall_timeout_s=fallback_stall,
            progress_eps_m=0.06,
        )
        return
    except Exception as exc:
        errs.append(f"B:{exc}")

    # Path C: one final clean retry after forcing a landed baseline again.
    try:
        bring_vehicle_to_baseline(px4)
        time.sleep(0.8)
        settle_ground()
        px4.command("commander arm -f")
        wait_armed(px4, timeout_s=5.0)
        time.sleep(0.55)
        enter_offboard_position_hold(px4, target_alt_m, timeout_s=min(8.0, remaining()))
        wait_until_altitude(
            px4,
            target_alt_m,
            min(primary_budget, remaining()),
            stall_timeout_s=primary_stall,
            progress_eps_m=0.06,
            completion_tol_m=0.14,
        )
        return
    except Exception as exc:
        errs.append(f"C:{exc}")

    raise RuntimeError(" / ".join(errs))


def wait_offboard_mode(px4: Px4Ctl, timeout_s: float = 10.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        out = px4.command("commander status", timeout_s=4.0)
        if "navigation mode: Offboard" in out:
            return
        time.sleep(0.2)
    raise RuntimeError("Vehicle did not enter Offboard mode")


def enter_offboard_position_hold(px4: Px4Ctl, target_alt_m: float, timeout_s: float = 10.0) -> None:
    t0 = time.time()
    target_z = -abs(target_alt_m)

    while time.time() - t0 < timeout_s:
        px4.command("trajectory_reader set_mode position")
        px4.command(f"trajectory_reader abs_ref 0 0 {target_z:.3f} 0")

        try:
            px4.command("commander mode offboard")
        except Exception:
            pass

        out = px4.command("commander status", timeout_s=4.0)
        if "navigation mode: Offboard" in out:
            return

        time.sleep(0.25)

    raise RuntimeError("Vehicle did not enter Offboard mode")


def set_reader_mode_with_retry(px4: Px4Ctl, mode_name: str, retries: int = 6) -> None:
    for _ in range(retries):
        px4.command(f"trajectory_reader set_mode {mode_name}", timeout_s=4.0)
        status = px4.command("trajectory_reader status", timeout_s=4.0)
        if f"Mode: {mode_name.upper()}" in status:
            return
        time.sleep(0.25)
    raise RuntimeError(f"Failed to switch trajectory_reader to {mode_name.upper()} mode")


def wait_trajectory_finished(px4: Px4Ctl,
                             timeout_s: float,
                             tracking_dir: Path,
                             traj_id: int,
                             mission_mode: str,
                             ident_profile: str,
                             run_begin: float,
                             live_trace_json: Optional[Path],
                             eval_id: int,
                             worker_id: int) -> tuple[str, float, Optional[Path]]:
    t0 = time.time()
    last_status = ""

    energy_integral = 0.0
    energy_time = 0.0
    last_t = time.time()

    last_live_update = 0.0
    current_log: Optional[Path] = None
    last_mode_recovery = 0.0
    disarm_seen_since: Optional[float] = None
    no_start_since: Optional[float] = None

    while time.time() - t0 < timeout_s:
        last_status = px4.command("trajectory_reader status", timeout_s=4.0)

        if "not running" in last_status.lower():
            raise RuntimeError("trajectory_reader is not running during trajectory execution")

        now = time.time()
        if "Mode: POSITION" in last_status and now - last_mode_recovery > 0.8:
            # OFFBOARD can sporadically force POSITION mode back in. Keep nudging the
            # trajectory reader into TRAJECTORY mode instead of giving up after a few tries,
            # otherwise the reader can stall part-way through the file and never reach EOF.
            try:
                px4.command("commander mode offboard", timeout_s=3.0)
            except Exception:
                pass
            try:
                set_reader_mode_with_retry(px4, mission_mode)
                last_mode_recovery = now
                time.sleep(0.2)
                continue
            except Exception:
                last_mode_recovery = now

        st = get_vehicle_status_fields(px4)
        if st.get("arming_state", -1) != 2:
            if disarm_seen_since is None:
                disarm_seen_since = now
            elif now - disarm_seen_since > 1.2:
                raise RuntimeError("Vehicle disarmed before trajectory finished")
        else:
            disarm_seen_since = None

        dt = max(0.0, min(0.3, now - last_t))
        last_t = now

        # Lower sampling load to reduce sim slowdown.
        if int((now - t0) * 4) != int((now - t0 - dt) * 4):
            thrust = get_attitude_thrust_cmd(px4)
            if thrust is not None:
                effort = abs(thrust)
                energy_integral += effort * effort * dt
                energy_time += dt

        if now - last_live_update > 0.7:
            cand = latest_tracking_log(tracking_dir, traj_id, run_begin, ident_profile=ident_profile)
            if cand is not None:
                current_log = cand
                write_live_trace(
                    live_trace_json,
                    eval_id,
                    current_log,
                    done=False,
                    message="running",
                    worker_id=worker_id,
                )
            last_live_update = now

        eof = parse_status_field(last_status, "EOF")
        log_open = parse_status_field(last_status, "Tracking log open")
        pending = parse_status_field(last_status, "Pending tracking samples")

        try:
            pending_n = int(pending) if pending is not None else None
        except ValueError:
            pending_n = None

        samples_txt = parse_status_field(last_status, "Samples read")
        samples_read = None
        samples_total = None
        if samples_txt is not None:
            m_samples = re.match(r"\s*(\d+)\s*/\s*(\d+)", samples_txt)
            if m_samples:
                samples_read = int(m_samples.group(1))
                samples_total = int(m_samples.group(2))

        if (
            samples_read == 0
            and samples_total == 0
            and eof == "no"
            and log_open == "no"
            and pending_n in (0, None)
        ):
            if no_start_since is None:
                no_start_since = now
            elif now - no_start_since > 3.0:
                raise RuntimeError("Trajectory did not start (samples 0/0)")
        else:
            no_start_since = None

        if eof == "yes" and log_open == "no" and pending_n in (0, None):
            energy_term = energy_integral / max(energy_time, 1e-6)
            write_live_trace(
                live_trace_json,
                eval_id,
                current_log,
                done=True,
                message="completed",
                worker_id=worker_id,
            )
            return last_status, energy_term, current_log

        time.sleep(0.25)

    raise RuntimeError("Trajectory timeout")


def ensure_dirs(rootfs: Path) -> None:
    params_dir = rootfs / "parameters"
    traj_dir = rootfs / "trajectories"
    log_dir = rootfs / "tracking_logs"

    params_dir.mkdir(parents=True, exist_ok=True)
    traj_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    if not os.access(traj_dir, os.W_OK):
        raise RuntimeError(
            f"{traj_dir} is not writable. "
            f"Run: sudo chown -R $USER:$USER {traj_dir}"
        )


@dataclass
class EvalResult:
    fitness: float
    track_rmse: float
    energy_term: float
    tracking_log: str
    identification_log: str
    params_file: str


def resolve_eval_logs_and_metrics(
    *,
    mission_mode: str,
    tracking_dir: Path,
    identification_dir: Path,
    traj_id: int,
    run_begin: float,
    ident_profile: str,
    tracking_log_from_loop: Optional[Path],
    trace_name: str,
    energy_term: float,
    w_track: float,
    w_energy: float,
) -> tuple[Optional[Path], Optional[Path], float, float]:
    tracking_log = tracking_log_from_loop or latest_tracking_log(
        tracking_dir,
        traj_id,
        run_begin,
        ident_profile=ident_profile if mission_mode == "identification" else "",
    )
    identification_log = (
        latest_identification_log(identification_dir, ident_profile, run_begin)
        if mission_mode == "identification"
        else None
    )

    if mission_mode == "identification":
        if identification_log is None and tracking_log is None:
            raise RuntimeError(
                f"No identification or tracking log found in {identification_dir} / {tracking_dir} "
                f"for ident_profile={ident_profile}"
            )
        track_rmse = compute_tracking_rmse(tracking_log, trace_name) if tracking_log is not None else 0.0
    else:
        if tracking_log is None:
            raise RuntimeError(f"No tracking log found in {tracking_dir} for traj_id={traj_id}")
        track_rmse = compute_tracking_rmse(tracking_log, trace_name)

    fitness = w_track * track_rmse + w_energy * energy_term
    return tracking_log, identification_log, track_rmse, fitness


def main() -> int:
    ap = argparse.ArgumentParser(description="Evaluate controller params on PX4 SITL trajectory")
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--controller", choices=("dfbc", "pid", "indi", "mpc", "cmpc", "sysid"), default="dfbc")
    ap.add_argument("--traj-id", type=int, default=0)
    ap.add_argument("--takeoff-alt", type=float, default=2.0)
    ap.add_argument("--trajectory-timeout", type=float, default=180.0)
    ap.add_argument("--takeoff-timeout", type=float, default=30.0)
    ap.add_argument("--w-track", type=float, default=1.0)
    ap.add_argument("--w-energy", type=float, default=0.05)
    ap.add_argument("--shell-endpoint", default="unused")
    ap.add_argument("--telemetry-endpoint", default="unused")
    ap.add_argument("--hard-reset-start", action="store_true")
    ap.add_argument("--hard-reset-end", action="store_true")
    ap.add_argument("--result-json", default="")
    ap.add_argument("--live-trace-json", default="")
    ap.add_argument("--eval-id", type=int, default=-1)
    ap.add_argument("--worker-id", type=int, default=-1)
    ap.add_argument("--instance-id", type=int, default=0)
    ap.add_argument("--relaxed-replay", action="store_true")
    ap.add_argument("--landing-mode", choices=("land", "rtl"), default="land")
    ap.add_argument("--trace-window", choices=("offboard",), default="offboard")
    ap.add_argument("--engagement-dwell-s", type=float, default=2.0)
    ap.add_argument("--mission-mode", choices=("trajectory", "identification"), default="trajectory")
    ap.add_argument("--ident-profile", default="hover_thrust")
    ap.add_argument("--base-param-file", default="")
    ap.add_argument("--fixed-param", action="append", default=[])
    args = ap.parse_args()

    if args.hard_reset_start or args.hard_reset_end:
        raise RuntimeError("hard reset flags are not supported in local-wrapper mode")

    rootfs = Path(args.rootfs).resolve()
    profile = get_controller_profile(args.controller)
    default_params_file = ensure_default_param_file(rootfs, profile)
    params_file = Path(args.params_file).resolve() if args.params_file else default_params_file
    tracking_dir = rootfs / "tracking_logs"
    identification_dir = rootfs / "identification_logs"
    live_trace_json = Path(args.live_trace_json).resolve() if args.live_trace_json else None

    ensure_dirs(rootfs)
    params = parse_param_file(params_file, profile)
    fixed_params = parse_fixed_param_items(args.fixed_param)
    px4 = Px4Ctl(rootfs, instance_id=args.instance_id)

    write_live_trace(
        live_trace_json,
        args.eval_id,
        None,
        done=False,
        message="starting",
        worker_id=args.worker_id,
    )

    # Bring vehicle to a known-safe baseline.
    bring_vehicle_to_baseline(px4)

    start_module_safe(px4, "custom_pos_control")
    start_module_safe(px4, "trajectory_reader")

    apply_base_param_dump(px4, args.base_param_file)

    # Stabilize preflight checks for repeated optimization cycles in SITL.
    apply_sitl_stability_params(px4)
    apply_runtime_speed_params(px4)
    apply_takeoff_alt_param(px4, args.takeoff_alt)

    prepare_target_controller(px4, profile, fixed_params, params)
    activate_px4_default_controller(px4)

    energy_term = 0.0
    log_path_from_loop: Optional[Path] = None
    last_exec_error: Optional[Exception] = None

    max_attempts = 3 if profile.name in {"pid", "indi"} else 2

    for attempt in range(max_attempts):
        run_begin = time.time()
        apply_sitl_stability_params(px4)
        apply_runtime_speed_params(px4)
        apply_takeoff_alt_param(px4, args.takeoff_alt)
        nav_ready_timeout = 28.0 if args.relaxed_replay else 12.0
        preflight_ready_timeout = 28.0 if args.relaxed_replay else 12.0
        if args.mission_mode == "trajectory":
            px4.command(f"trajectory_reader set_traj_id {args.traj_id}")
        else:
            px4.command(f"trajectory_reader set_ident_profile {args.ident_profile}")
        px4.command("trajectory_reader set_mode position")
        px4.command("trajectory_reader ref 0 0 -2.0 0")

        try:
            if not wait_navigation_ready(px4, timeout_s=nav_ready_timeout):
                raise RuntimeError("Navigation estimator not ready (local position invalid/dead_reckoning)")

            try:
                wait_ground_stable(px4)
            except RuntimeError:
                # SITL can keep tiny residual ground-motion estimates after the previous
                # landing cycle. If we are already landed and disarmed, do not block
                # the next evaluation on an overly strict ground gate.
                if get_vehicle_status_fields(px4).get("arming_state", -1) == 1 and is_landed(px4):
                    pass
                else:
                    raise

            # In SITL the commander can keep transient warnings active even though
            # arm -f and takeoff are still accepted. Treat this as advisory and let
            # the actual arm/takeoff sequence determine success.
            wait_preflight_ready(px4, timeout_s=preflight_ready_timeout)

            activate_px4_default_controller(px4)
            takeoff_with_retry(px4, args.takeoff_alt, args.takeoff_timeout, profile.name)

            # Hold the hover point under PX4 default first so ground-to-air
            # transition is controller-agnostic and repeatable.
            px4.command("trajectory_reader set_mode position")
            px4.command(f"trajectory_reader abs_ref 0 0 {-abs(args.takeoff_alt):.3f} 0")
            time.sleep(0.2)
            initial_gate = initial_hover_profile(profile.name, relaxed=args.relaxed_replay)
            try:
                wait_hover_stable(
                    px4,
                    args.takeoff_alt,
                    **initial_gate,
                )
            except RuntimeError:
                if args.relaxed_replay:
                    raise
                px4.command(f"trajectory_reader abs_ref 0 0 {-abs(args.takeoff_alt):.3f} 0")
                wait_hover_stable(
                    px4,
                    args.takeoff_alt,
                    **relaxed_hover_profile(initial_gate),
                )
            try:
                wait_hover_thrust_valid(px4, timeout_s=5.0)
            except RuntimeError:
                if not args.relaxed_replay:
                    raise

            activate_target_controller(px4, profile, fixed_params, params)
            enter_offboard_position_hold(px4, args.takeoff_alt, timeout_s=10.0)
            # OFFBOARD entry forces trajectory_reader back to POSITION with a zero target.
            # Reapply an absolute hover hold after mode transition to remove state-dependent drift.
            time.sleep(0.25)
            px4.command(f"trajectory_reader abs_ref 0 0 {-abs(args.takeoff_alt):.3f} 0")
            try:
                engagement_gate = hover_stability_profile(profile.name, engagement_dwell_s=args.engagement_dwell_s)
                if args.relaxed_replay:
                    engagement_gate = relaxed_engagement_profile(engagement_gate)
                wait_controller_engaged(px4, args.takeoff_alt, **engagement_gate)
            except RuntimeError:
                if args.relaxed_replay:
                    raise
                if not allow_relaxed_hover_gate(profile.name):
                    raise
                time.sleep(1.5)
                px4.command(f"trajectory_reader abs_ref 0 0 {-abs(args.takeoff_alt):.3f} 0")
                relaxed_gate = relaxed_engagement_profile(engagement_gate)
                relaxed_gate["timeout_s"] = max(float(relaxed_gate.get("timeout_s", 0.0)), 22.0)
                wait_controller_engaged(px4, args.takeoff_alt, **relaxed_gate)
            try:
                wait_hover_thrust_valid(px4, timeout_s=5.0)
            except RuntimeError:
                if not args.relaxed_replay:
                    raise
            px4.command(f"trajectory_reader set_traj_anchor 0 0 {-abs(args.takeoff_alt):.3f}")
            set_reader_mode_with_retry(px4, args.mission_mode)

            _status, energy_term, log_path_from_loop = wait_trajectory_finished(
                px4,
                args.trajectory_timeout,
                tracking_dir,
                args.traj_id,
                args.mission_mode,
                args.ident_profile,
                run_begin,
                live_trace_json,
                args.eval_id,
                args.worker_id,
            )
            last_exec_error = None
            break
        except Exception as exc:
            last_exec_error = exc
            bring_vehicle_to_baseline(px4)

            try:
                px4.command("trajectory_reader stop")
            except Exception:
                pass

            try:
                px4.command("trajectory_reader start")
            except Exception:
                start_module_safe(px4, "trajectory_reader")

            start_module_safe(px4, "custom_pos_control")
            prepare_target_controller(px4, profile, fixed_params, params)
            activate_px4_default_controller(px4)
            apply_sitl_stability_params(px4)
            apply_runtime_speed_params(px4)
            apply_takeoff_alt_param(px4, args.takeoff_alt)

            retryable_bootstrap = (
                profile.name in {"pid", "indi"}
                and any(token in str(exc).lower() for token in ("takeoff", "preflight", "stabilize", "offboard"))
            )

            if attempt < (max_attempts - 1) and (should_retry_eval(exc) or retryable_bootstrap):
                continue

            raise

    if last_exec_error is not None:
        raise last_exec_error

    try:
        if str(args.landing_mode).strip().lower() == "rtl":
            px4.command("commander mode auto:rtl")
        else:
            px4.command("commander mode auto:land")
    except Exception:
        pass
    try:
        wait_disarmed_and_landed(px4, timeout_s=20.0)
    except Exception:
        px4.command("commander disarm -f")

    log_path, identification_log, track_rmse, fitness = resolve_eval_logs_and_metrics(
        mission_mode=args.mission_mode,
        tracking_dir=tracking_dir,
        identification_dir=identification_dir,
        traj_id=args.traj_id,
        run_begin=run_begin,
        ident_profile=args.ident_profile,
        tracking_log_from_loop=log_path_from_loop,
        trace_name=profile.trace_name,
        energy_term=energy_term,
        w_track=args.w_track,
        w_energy=args.w_energy,
    )

    result = EvalResult(
        fitness=fitness,
        track_rmse=track_rmse,
        energy_term=energy_term,
        tracking_log=str(log_path),
        identification_log=str(identification_log or ""),
        params_file=str(params_file),
    )

    payload = {
        "fitness": result.fitness,
        "track_rmse": result.track_rmse,
        "energy_term": result.energy_term,
        "tracking_log": result.tracking_log,
        "identification_log": result.identification_log,
        "params_file": result.params_file,
        "traj_id": args.traj_id,
        "eval_id": args.eval_id,
        "controller": profile.name,
        "fixed_params": fixed_params,
        "trace_window": str(args.trace_window),
        "landing_mode": str(args.landing_mode),
        "mission_mode": str(args.mission_mode),
        "ident_profile": str(args.ident_profile),
    }

    if args.result_json:
        out = Path(args.result_json)
        atomic_write_text(out, json.dumps(payload, indent=2))

    write_live_trace(
        live_trace_json,
        args.eval_id,
        log_path,
        done=True,
        message="completed",
        worker_id=args.worker_id,
        fitness=fitness,
    )
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        # Best effort dashboard update on failure.
        try:
            argv = os.sys.argv
            if "--live-trace-json" in argv:
                i = argv.index("--live-trace-json")
                p = Path(argv[i + 1]).resolve() if i + 1 < len(argv) and argv[i + 1] else None
                eval_id = -1
                worker_id = -1
                if "--eval-id" in argv:
                    j = argv.index("--eval-id")
                    if j + 1 < len(argv):
                        eval_id = int(argv[j + 1])
                if "--worker-id" in argv:
                    k = argv.index("--worker-id")
                    if k + 1 < len(argv):
                        worker_id = int(argv[k + 1])
                if p:
                    write_live_trace(
                        p,
                        eval_id,
                        None,
                        done=True,
                        message=f"failed: {exc}",
                        worker_id=worker_id,
                    )
        except Exception:
            pass

        print(json.dumps({"error": str(exc)}), file=os.sys.stderr)
        raise
