#!/usr/bin/env python3
"""Controller tuning profiles and parameter helpers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np


@dataclass(frozen=True)
class ParamSpec:
    name: str
    default: float
    safe_bounds: Tuple[float, float]
    wide_bounds: Tuple[float, float]
    kind: str = "float"


@dataclass(frozen=True)
class ControllerProfile:
    name: str
    cli_name: str
    trace_name: str
    display_name: str
    controller_id: int
    param_file_name: str
    params: Tuple[ParamSpec, ...]


DFBC_PROFILE = ControllerProfile(
    name="dfbc",
    cli_name="dfbc",
    trace_name="dfbc",
    display_name="DFBC",
    controller_id=1,
    param_file_name="dfbc_parameters",
    params=(
        ParamSpec("DFBC_KR_X", 3.2, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KR_Y", 3.2, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KR_Z", 3.2, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KV_X", 2.1, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KV_Y", 2.1, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KV_Z", 2.1, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("DFBC_KQ_X", 50.1, (5.0, 120.0), (0.5, 200.0)),
        ParamSpec("DFBC_KQ_Y", 50.1, (5.0, 120.0), (0.5, 200.0)),
        ParamSpec("DFBC_KQ_Z", 3.1, (0.5, 20.0), (0.1, 50.0)),
        ParamSpec("DFBC_W_MIN", -6.0, (-12.0, -0.5), (-20.0, -0.1)),
        ParamSpec("DFBC_W_MAX", 6.0, (0.5, 12.0), (0.1, 20.0)),
        ParamSpec("DFBC_MIN_THR", 0.6, (0.20, 0.85), (0.10, 1.20)),
        ParamSpec("DFBC_MAX_THR", 1.4, (1.05, 1.80), (0.80, 2.00)),
        ParamSpec("DFBC_MAX_TILT", 50.0, (35.0, 60.0), (20.0, 60.0)),
    ),
)

MPC_PROFILE = ControllerProfile(
    name="mpc",
    cli_name="mpc",
    trace_name="mpc",
    display_name="MPC",
    controller_id=2,
    param_file_name="mpc_parameters",
    params=(
        ParamSpec("MPC_QX", 2000.0, (200.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_QY", 2000.0, (200.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_QZ", 2000.0, (200.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_QVX", 1800.0, (100.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_QVY", 1800.0, (100.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_QVZ", 1800.0, (100.0, 4000.0), (10.0, 10000.0)),
        ParamSpec("MPC_RX", 20.0, (0.5, 200.0), (0.01, 500.0)),
        ParamSpec("MPC_RY", 20.0, (0.5, 200.0), (0.01, 500.0)),
        ParamSpec("MPC_RZ", 20.0, (0.5, 200.0), (0.01, 500.0)),
    ),
)

CMPC_PROFILE = ControllerProfile(
    name="cmpc",
    cli_name="cmpc",
    trace_name="cmpc",
    display_name="CMPC",
    controller_id=3,
    param_file_name="cmpc_parameters",
    params=(
        ParamSpec("CMPC_QX_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QY_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QZ_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QX_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_QY_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_QZ_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_QVX_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QVY_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QVZ_MIN", 600.0, (50.0, 1500.0), (1.0, 5000.0)),
        ParamSpec("CMPC_QVX_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_QVY_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_QVZ_MAX", 1000.0, (100.0, 2500.0), (1.0, 6000.0)),
        ParamSpec("CMPC_SIGNAL_W0", 0.0, (0.0, 10.0), (0.0, 20.0), kind="int"),
        ParamSpec("CMPC_SIGNAL_WR", 2.0, (0.0, 10.0), (0.0, 20.0), kind="int"),
        ParamSpec("CMPC_SIGNAL_WL", 5.0, (1.0, 15.0), (1.0, 30.0), kind="int"),
        ParamSpec("CMPC_SIGNAL_WD", 2.0, (0.0, 10.0), (0.0, 20.0), kind="int"),
    ),
)

PID_PROFILE = ControllerProfile(
    name="pid",
    cli_name="px4_default",
    trace_name="px4_default",
    display_name="PX4 PID",
    controller_id=4,
    param_file_name="pid_parameters",
    params=(
        ParamSpec("MPC_XY_P", 0.95, (0.40, 1.50), (0.10, 2.00)),
        ParamSpec("MPC_Z_P", 1.00, (0.40, 1.50), (0.10, 2.00)),
        ParamSpec("MPC_XY_VEL_P_ACC", 1.80, (1.20, 4.50), (0.50, 6.00)),
        ParamSpec("MPC_XY_VEL_I_ACC", 0.40, (0.00, 6.00), (0.00, 20.00)),
        ParamSpec("MPC_XY_VEL_D_ACC", 0.20, (0.00, 1.20), (0.00, 2.00)),
        ParamSpec("MPC_Z_VEL_P_ACC", 4.00, (2.00, 10.00), (1.00, 15.00)),
        ParamSpec("MPC_Z_VEL_I_ACC", 2.00, (0.20, 3.50), (0.00, 5.00)),
        ParamSpec("MPC_Z_VEL_D_ACC", 0.00, (0.00, 0.80), (0.00, 2.00)),
        ParamSpec("MPC_TILTMAX_AIR", 45.00, (25.00, 60.00), (20.00, 85.00)),
        ParamSpec("MPC_THR_HOVER", 0.50, (0.30, 0.70), (0.10, 0.80)),
    ),
)

INDI_PROFILE = ControllerProfile(
    name="indi",
    cli_name="indi",
    trace_name="indi",
    display_name="INDI",
    controller_id=5,
    param_file_name="indi_parameters",
    params=(
        ParamSpec("INDI_KR_X", 3.2, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_KR_Y", 3.2, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_KR_Z", 2.5, (0.5, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_KV_X", 1.8, (0.3, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_KV_Y", 1.8, (0.3, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_KV_Z", 1.4, (0.3, 8.0), (0.1, 15.0)),
        ParamSpec("INDI_ATT_P_X", 18.0, (1.0, 32.0), (0.5, 120.0)),
        ParamSpec("INDI_ATT_P_Y", 18.0, (1.0, 32.0), (0.5, 120.0)),
        ParamSpec("INDI_ATT_P_Z", 8.0, (0.2, 12.0), (0.1, 24.0)),
        ParamSpec("INDI_RATE_D_X", 0.10, (0.02, 0.55), (0.01, 3.0)),
        ParamSpec("INDI_RATE_D_Y", 0.10, (0.02, 0.55), (0.01, 3.0)),
        ParamSpec("INDI_RATE_D_Z", 0.10, (0.01, 0.45), (0.005, 2.0)),
        ParamSpec("INDI_G1_X", 0.00, (0.00, 0.80), (0.00, 4.00)),
        ParamSpec("INDI_G1_Y", 0.00, (0.00, 0.80), (0.00, 4.00)),
        ParamSpec("INDI_G1_Z", 0.00, (0.00, 0.50), (0.00, 3.00)),
        ParamSpec("INDI_W_MAX", 8.0, (0.8, 6.0), (0.2, 14.0)),
        ParamSpec("INDI_MIN_THR", 0.35, (0.20, 0.85), (0.10, 1.20)),
        ParamSpec("INDI_MAX_THR", 1.50, (1.05, 1.80), (0.80, 2.00)),
        ParamSpec("INDI_MAX_TILT", 55.0, (20.0, 55.0), (15.0, 70.0)),
    ),
)

SYSID_PROFILE = ControllerProfile(
    name="sysid",
    cli_name="sysid",
    trace_name="sysid",
    display_name="System ID",
    controller_id=6,
    param_file_name="sysid_parameters",
    params=(),
)

PROFILES: Dict[str, ControllerProfile] = {
    DFBC_PROFILE.name: DFBC_PROFILE,
    PID_PROFILE.name: PID_PROFILE,
    INDI_PROFILE.name: INDI_PROFILE,
    MPC_PROFILE.name: MPC_PROFILE,
    CMPC_PROFILE.name: CMPC_PROFILE,
    SYSID_PROFILE.name: SYSID_PROFILE,
}


def describe_param(profile_name: str, param_name: str) -> str:
    profile = str(profile_name or "").strip().lower()
    name = str(param_name or "").strip().upper()

    axis = ""
    if name.endswith("_X") or name.endswith("X"):
        axis = " on the X axis"
    elif name.endswith("_Y") or name.endswith("Y"):
        axis = " on the Y axis"
    elif name.endswith("_Z") or name.endswith("Z"):
        axis = " on the Z axis"

    if profile == "dfbc":
        if name.startswith("DFBC_KR_"):
            return f"Outer-loop position error gain{axis}. Higher values react more aggressively to position mismatch."
        if name.startswith("DFBC_KV_"):
            return f"Outer-loop velocity error gain{axis}. Higher values damp motion faster but can make the response sharper."
        if name.startswith("DFBC_KQ_"):
            return f"Attitude tracking gain{axis}. It shapes how strongly DFBC corrects attitude error."
        if name == "DFBC_W_MIN":
            return "Lower clamp for commanded body-rate magnitude. More negative values allow stronger reverse angular motion."
        if name == "DFBC_W_MAX":
            return "Upper clamp for commanded body-rate magnitude. Higher values allow more aggressive angular motion."
        if name == "DFBC_MIN_THR":
            return "Minimum normalized thrust limit applied by the controller."
        if name == "DFBC_MAX_THR":
            return "Maximum normalized thrust limit applied by the controller."
        if name == "DFBC_MAX_TILT":
            return "Maximum allowed tilt angle in degrees during flight."

    if profile == "pid":
        if name == "MPC_XY_P":
            return "Position-loop proportional gain in the horizontal plane."
        if name == "MPC_Z_P":
            return "Position-loop proportional gain on altitude."
        if name == "MPC_XY_VEL_P_ACC":
            return "Horizontal velocity-loop proportional gain that maps velocity error to acceleration demand."
        if name == "MPC_XY_VEL_I_ACC":
            return "Horizontal velocity-loop integral gain. It removes steady-state bias but too much can cause oscillation."
        if name == "MPC_XY_VEL_D_ACC":
            return "Horizontal velocity-loop derivative gain. It damps fast changes in velocity error."
        if name == "MPC_Z_VEL_P_ACC":
            return "Vertical velocity-loop proportional gain."
        if name == "MPC_Z_VEL_I_ACC":
            return "Vertical velocity-loop integral gain."
        if name == "MPC_Z_VEL_D_ACC":
            return "Vertical velocity-loop derivative gain."
        if name == "MPC_TILTMAX_AIR":
            return "Maximum air-mode tilt angle in degrees."
        if name == "MPC_THR_HOVER":
            return "Normalized hover thrust estimate used by PX4."

    if profile == "indi":
        if name.startswith("INDI_KR_"):
            return f"Outer-loop position gain{axis} for the INDI controller."
        if name.startswith("INDI_KV_"):
            return f"Outer-loop velocity gain{axis} for the INDI controller."
        if name.startswith("INDI_ATT_P_"):
            return f"Attitude-to-rate proportional gain{axis}."
        if name.startswith("INDI_RATE_D_"):
            return f"Angular-rate damping from gyro derivative{axis}. Larger values suppress jitter but may slow response."
        if name.startswith("INDI_G1_"):
            return f"Control-effectiveness term{axis}. Higher values make INDI respond more strongly to angular-acceleration error."
        if name == "INDI_W_MAX":
            return "Maximum commanded body-rate magnitude."
        if name == "INDI_MIN_THR":
            return "Minimum normalized thrust limit."
        if name == "INDI_MAX_THR":
            return "Maximum normalized thrust limit."
        if name == "INDI_MAX_TILT":
            return "Maximum allowed tilt angle in degrees."

    if profile == "mpc":
        if name.startswith("MPC_Q"):
            return "State-cost weight in the MPC objective. Higher values prioritize tracking this state more strongly."
        if name.startswith("MPC_R"):
            return "Input-cost weight in the MPC objective. Higher values penalize aggressive control effort."

    if profile == "cmpc":
        if name.startswith("CMPC_Q") and name.endswith("_MIN"):
            return "Lower bound of the adaptive MPC state-cost schedule for this channel."
        if name.startswith("CMPC_Q") and name.endswith("_MAX"):
            return "Upper bound of the adaptive MPC state-cost schedule for this channel."
        if name == "CMPC_SIGNAL_W0":
            return "Center frequency or base shaping point used by the adaptive CMPC weight schedule."
        if name == "CMPC_SIGNAL_WR":
            return "Right-side width of the CMPC signal-shaping schedule."
        if name == "CMPC_SIGNAL_WL":
            return "Left-side width of the CMPC signal-shaping schedule."
        if name == "CMPC_SIGNAL_WD":
            return "Damping or transition width of the CMPC signal-shaping schedule."

    if profile == "sysid":
        return "System-identification forwarding controller. It relies on built-in excitation profiles instead of tunable gains."

    return "Controller-specific tuning parameter used by the optimization framework."

MPC_SHARED_QR_PARAMS: Tuple[str, ...] = (
    "MPC_QX",
    "MPC_QY",
    "MPC_QZ",
    "MPC_QVX",
    "MPC_QVY",
    "MPC_QVZ",
    "MPC_RX",
    "MPC_RY",
    "MPC_RZ",
)


def get_controller_profile(name: str) -> ControllerProfile:
    key = name.strip().lower()
    if key not in PROFILES:
        raise RuntimeError(f"Unknown controller profile: {name}")
    return PROFILES[key]


def all_profiles() -> Tuple[ControllerProfile, ...]:
    return tuple(PROFILES.values())


def param_names(profile: ControllerProfile) -> List[str]:
    return [p.name for p in profile.params]


def default_params(profile: ControllerProfile) -> Dict[str, float]:
    return {p.name: float(p.default) for p in profile.params}


def bounds_for_profile(profile: ControllerProfile, profile_name: str) -> Dict[str, Tuple[float, float]]:
    use_safe = profile_name == "safe"
    return {
        p.name: (p.safe_bounds if use_safe else p.wide_bounds)
        for p in profile.params
    }


def param_spec_map(profile: ControllerProfile) -> Dict[str, ParamSpec]:
    return {p.name: p for p in profile.params}


def clamp(bounds: Dict[str, Tuple[float, float]], name: str, value: float) -> float:
    lo, hi = bounds[name]
    return max(lo, min(hi, value))


def _normalize_numeric(profile: ControllerProfile, params: Dict[str, float]) -> Dict[str, float]:
    specs = param_spec_map(profile)
    normalized: Dict[str, float] = {}
    for name in param_names(profile):
        value = float(params[name])
        if specs[name].kind == "int":
            value = float(int(round(value)))
        normalized[name] = value
    return normalized


def apply_param_rules(
    profile: ControllerProfile,
    params: Dict[str, float],
    bounds: Dict[str, Tuple[float, float]],
) -> Dict[str, float]:
    out = _normalize_numeric(profile, params)
    for name in param_names(profile):
        out[name] = clamp(bounds, name, out[name])

    if "DFBC_MAX_THR" in out and "DFBC_MIN_THR" in out:
        out["DFBC_MAX_THR"] = max(out["DFBC_MAX_THR"], out["DFBC_MIN_THR"] + 0.25)
        out["DFBC_MAX_THR"] = clamp(bounds, "DFBC_MAX_THR", out["DFBC_MAX_THR"])
        out["DFBC_MIN_THR"] = min(out["DFBC_MIN_THR"], out["DFBC_MAX_THR"] - 0.20)
        out["DFBC_MIN_THR"] = clamp(bounds, "DFBC_MIN_THR", out["DFBC_MIN_THR"])
        out["DFBC_W_MAX"] = max(out["DFBC_W_MAX"], out["DFBC_W_MIN"] + 0.5)
        out["DFBC_W_MAX"] = clamp(bounds, "DFBC_W_MAX", out["DFBC_W_MAX"])
        out["DFBC_W_MIN"] = min(out["DFBC_W_MIN"], out["DFBC_W_MAX"] - 0.5)
        out["DFBC_W_MIN"] = clamp(bounds, "DFBC_W_MIN", out["DFBC_W_MIN"])

        kr_mean = (out["DFBC_KR_X"] + out["DFBC_KR_Y"] + out["DFBC_KR_Z"]) / 3.0
        kv_mean = (out["DFBC_KV_X"] + out["DFBC_KV_Y"] + out["DFBC_KV_Z"]) / 3.0
        if kv_mean > 1.9 * kr_mean:
            scale = (1.9 * kr_mean) / max(1e-6, kv_mean)
            for key in ("DFBC_KV_X", "DFBC_KV_Y", "DFBC_KV_Z"):
                out[key] = clamp(bounds, key, out[key] * scale)

    if "INDI_MAX_THR" in out and "INDI_MIN_THR" in out:
        out["INDI_MAX_THR"] = max(out["INDI_MAX_THR"], out["INDI_MIN_THR"] + 0.25)
        out["INDI_MAX_THR"] = clamp(bounds, "INDI_MAX_THR", out["INDI_MAX_THR"])
        out["INDI_MIN_THR"] = min(out["INDI_MIN_THR"], out["INDI_MAX_THR"] - 0.20)
        out["INDI_MIN_THR"] = clamp(bounds, "INDI_MIN_THR", out["INDI_MIN_THR"])

        kr_mean = (out["INDI_KR_X"] + out["INDI_KR_Y"] + out["INDI_KR_Z"]) / 3.0
        kv_mean = (out["INDI_KV_X"] + out["INDI_KV_Y"] + out["INDI_KV_Z"]) / 3.0
        if kv_mean > 1.9 * kr_mean:
            scale = (1.9 * kr_mean) / max(1e-6, kv_mean)
            for key in ("INDI_KV_X", "INDI_KV_Y", "INDI_KV_Z"):
                out[key] = clamp(bounds, key, out[key] * scale)

        g1_mean = (out["INDI_G1_X"] + out["INDI_G1_Y"] + out["INDI_G1_Z"]) / 3.0
        if g1_mean > 0.30:
            for key in ("INDI_ATT_P_X", "INDI_ATT_P_Y"):
                out[key] = clamp(bounds, key, min(out[key], 24.0))
            for key in ("INDI_RATE_D_X", "INDI_RATE_D_Y", "INDI_RATE_D_Z"):
                out[key] = clamp(bounds, key, min(out[key], 0.35))
            out["INDI_W_MAX"] = clamp(bounds, "INDI_W_MAX", min(out["INDI_W_MAX"], 5.5))

    if "MPC_MAX_THR" in out and "MPC_MIN_THR" in out:
        out["MPC_MAX_THR"] = max(out["MPC_MAX_THR"], out["MPC_MIN_THR"] + 0.25)
        out["MPC_MAX_THR"] = clamp(bounds, "MPC_MAX_THR", out["MPC_MAX_THR"])
        out["MPC_MIN_THR"] = min(out["MPC_MIN_THR"], out["MPC_MAX_THR"] - 0.20)
        out["MPC_MIN_THR"] = clamp(bounds, "MPC_MIN_THR", out["MPC_MIN_THR"])

    if profile.name == "cmpc":
        for axis in ("X", "Y", "Z"):
            min_key = f"CMPC_Q{axis}_MIN"
            max_key = f"CMPC_Q{axis}_MAX"
            out[max_key] = max(out[max_key], out[min_key] + 10.0)
            out[max_key] = clamp(bounds, max_key, out[max_key])
            out[min_key] = min(out[min_key], out[max_key] - 10.0)
            out[min_key] = clamp(bounds, min_key, out[min_key])

            vmin_key = f"CMPC_QV{axis}_MIN"
            vmax_key = f"CMPC_QV{axis}_MAX"
            out[vmax_key] = max(out[vmax_key], out[vmin_key] + 10.0)
            out[vmax_key] = clamp(bounds, vmax_key, out[vmax_key])
            out[vmin_key] = min(out[vmin_key], out[vmax_key] - 10.0)
            out[vmin_key] = clamp(bounds, vmin_key, out[vmin_key])

        for name in ("CMPC_SIGNAL_W0", "CMPC_SIGNAL_WR", "CMPC_SIGNAL_WL", "CMPC_SIGNAL_WD"):
            out[name] = float(int(round(out[name])))

    return out


def mpc_shared_qr_defaults() -> Dict[str, float]:
    return {name: default_params(MPC_PROFILE)[name] for name in MPC_SHARED_QR_PARAMS}


def parse_param_file(path: Path, profile: ControllerProfile) -> Dict[str, float]:
    wanted = set(param_names(profile))
    params: Dict[str, float] = {}
    if not path.exists():
        raise RuntimeError(f"Parameter file not found: {path}")

    for line in path.read_text(encoding="utf-8").splitlines():
        raw = line.strip()
        if not raw or raw.startswith("#"):
            continue
        if "=" in raw:
            key, value = raw.split("=", 1)
        else:
            parts = raw.split()
            if len(parts) != 2:
                continue
            key, value = parts
        key = key.strip()
        if key not in wanted:
            continue
        params[key] = float(value.strip())

    missing = [name for name in param_names(profile) if name not in params]
    if missing:
        raise RuntimeError(f"Missing {profile.display_name} params in {path}: {missing}")

    return _normalize_numeric(profile, params)


def write_param_file(path: Path, profile: ControllerProfile, params: Dict[str, float], header: str = "") -> None:
    normalized = _normalize_numeric(profile, params)
    lines = []
    if header:
        lines.append(f"# {header}")
    lines.append("# Format: PARAM_NAME=VALUE")
    lines.append("")
    for spec in profile.params:
        value = normalized[spec.name]
        if spec.kind == "int":
            lines.append(f"{spec.name}={int(round(value))}")
        else:
            lines.append(f"{spec.name}={value:.8f}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ensure_default_param_file(rootfs: Path, profile: ControllerProfile) -> Path:
    path = rootfs / "parameters" / profile.param_file_name
    defaults = default_params(profile)
    if not path.exists():
        write_param_file(
            path,
            profile,
            defaults,
            header=f"{profile.display_name} parameter set loaded by controller_profiles.py",
        )
        return path

    try:
        current = parse_param_file(path, profile)
    except Exception:
        current = {}

    merged = dict(defaults)
    merged.update(current)
    if merged != current:
        write_param_file(
            path,
            profile,
            merged,
            header=f"{profile.display_name} parameter set loaded by controller_profiles.py",
        )
    return path


def params_to_vec(profile: ControllerProfile, params: Dict[str, float]) -> np.ndarray:
    return np.array([float(params[name]) for name in param_names(profile)], dtype=float)


def vec_to_params(profile: ControllerProfile, vec: np.ndarray) -> Dict[str, float]:
    return {name: float(vec[i]) for i, name in enumerate(param_names(profile))}


def bounds_array(
    profile: ControllerProfile,
    bounds: Dict[str, Tuple[float, float]],
) -> Tuple[np.ndarray, np.ndarray]:
    names = param_names(profile)
    lo = np.array([bounds[name][0] for name in names], dtype=float)
    hi = np.array([bounds[name][1] for name in names], dtype=float)
    return lo, hi


def finite_diff_delta(spec: ParamSpec, center: float, fd_eps: float) -> float:
    if spec.kind == "int":
        return 1.0
    return float(fd_eps) * max(1.0, abs(center))


def profile_from_controller_value(value: str) -> ControllerProfile:
    return get_controller_profile(value)


def list_profile_names() -> Iterable[str]:
    return PROFILES.keys()
