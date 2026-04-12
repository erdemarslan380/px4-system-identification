#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from copy import deepcopy
from pathlib import Path


JMAVSIM_ARM_LENGTH_M = 0.33 / 2.0
JMAVSIM_DIAG_COORD_M = JMAVSIM_ARM_LENGTH_M / math.sqrt(2.0)
ROTOR_Z_M = 0.06


def _x500_scaled_positions() -> dict[str, list[float]]:
    d = JMAVSIM_DIAG_COORD_M
    return {
        "rotor_0": [d, -d, ROTOR_Z_M],
        "rotor_1": [-d, d, ROTOR_Z_M],
        "rotor_2": [d, d, ROTOR_Z_M],
        "rotor_3": [-d, -d, ROTOR_Z_M],
    }


def _jmavsim_direct_positions() -> dict[str, list[float]]:
    d = JMAVSIM_DIAG_COORD_M
    return {
        "rotor_0": [d, d, ROTOR_Z_M],
        "rotor_1": [-d, -d, ROTOR_Z_M],
        "rotor_2": [d, -d, ROTOR_Z_M],
        "rotor_3": [-d, d, ROTOR_Z_M],
    }


GEOMETRY_PRESETS: dict[str, dict] = {
    "x500_scaled_165mm": {
        "label": "x500 scaled to jMAVSim 165 mm arm length",
        "rotor_positions_m": _x500_scaled_positions(),
        "turning_directions": {
            "motor_0": "ccw",
            "motor_1": "ccw",
            "motor_2": "cw",
            "motor_3": "cw",
        },
    },
    "jmavsim_direct_keep_spin": {
        "label": "jMAVSim direct rotor-index placement, keep existing spin assignment",
        "rotor_positions_m": _jmavsim_direct_positions(),
        "turning_directions": {
            "motor_0": "ccw",
            "motor_1": "ccw",
            "motor_2": "cw",
            "motor_3": "cw",
        },
    },
    "jmavsim_direct_swap_spin": {
        "label": "jMAVSim direct rotor-index placement, swap diagonal spin assignment",
        "rotor_positions_m": _jmavsim_direct_positions(),
        "turning_directions": {
            "motor_0": "cw",
            "motor_1": "cw",
            "motor_2": "ccw",
            "motor_3": "ccw",
        },
    },
}


def _load_candidate(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _scale_motor_value(payload: dict, key: str, scale: float) -> None:
    node = payload["motor_model"][key]
    node["value"] = float(node["value"]) * float(scale)


def build_variant(
    *,
    base_payload: dict,
    geometry_preset: str,
    time_constant_up_scale: float,
    time_constant_down_scale: float,
    motor_constant_scale: float,
    moment_constant_scale: float,
    rotor_drag_scale: float,
    rolling_moment_scale: float,
    slowdown_scale: float,
    body_linear_decay: float = 0.0,
    body_angular_decay: float = 0.0,
    body_drag_scale_x: float = 1.0,
    body_drag_scale_y: float = 1.0,
    body_drag_scale_z: float = 1.0,
    body_angular_damping_x: float = 0.0,
    body_angular_damping_y: float = 0.0,
    body_angular_damping_z: float = 0.0,
    motor_balance_x: float = 0.0,
    motor_balance_y: float = 0.0,
    moment_balance_x: float = 0.0,
    moment_balance_y: float = 0.0,
) -> dict:
    payload = deepcopy(base_payload)
    payload["geometry"] = deepcopy(GEOMETRY_PRESETS[geometry_preset])
    payload.setdefault("warnings", []).append(f"Bridge calibration geometry preset: {geometry_preset}")
    payload.setdefault("warnings", []).append(
        "Bridge residual calibration candidate: simulator-only geometry/motor overrides for HITL-to-SITL matching."
    )
    payload.setdefault("composite_sources", {})["geometry"] = f"preset:{geometry_preset}"

    _scale_motor_value(payload, "time_constant_up_s", time_constant_up_scale)
    _scale_motor_value(payload, "time_constant_down_s", time_constant_down_scale)
    _scale_motor_value(payload, "motor_constant", motor_constant_scale)
    _scale_motor_value(payload, "moment_constant", moment_constant_scale)
    _scale_motor_value(payload, "rotor_drag_coefficient", rotor_drag_scale)
    _scale_motor_value(payload, "rolling_moment_coefficient", rolling_moment_scale)
    _scale_motor_value(payload, "rotor_velocity_slowdown_sim", slowdown_scale)
    payload["simulator_residuals"] = {
        "body_velocity_decay_linear": float(body_linear_decay),
        "body_velocity_decay_angular": float(body_angular_decay),
        "body_drag_scale_x": float(body_drag_scale_x),
        "body_drag_scale_y": float(body_drag_scale_y),
        "body_drag_scale_z": float(body_drag_scale_z),
        "body_angular_damping_x": float(body_angular_damping_x),
        "body_angular_damping_y": float(body_angular_damping_y),
        "body_angular_damping_z": float(body_angular_damping_z),
        "motor_balance_x": float(motor_balance_x),
        "motor_balance_y": float(motor_balance_y),
        "moment_balance_x": float(moment_balance_x),
        "moment_balance_y": float(moment_balance_y),
    }
    return payload


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Materialize a bridge-calibration candidate variant from an identified base candidate.")
    ap.add_argument("--base-candidate", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--geometry-preset", required=True, choices=sorted(GEOMETRY_PRESETS))
    ap.add_argument("--time-constant-up-scale", type=float, default=1.0)
    ap.add_argument("--time-constant-down-scale", type=float, default=1.0)
    ap.add_argument("--motor-constant-scale", type=float, default=1.0)
    ap.add_argument("--moment-constant-scale", type=float, default=1.0)
    ap.add_argument("--rotor-drag-scale", type=float, default=1.0)
    ap.add_argument("--rolling-moment-scale", type=float, default=1.0)
    ap.add_argument("--slowdown-scale", type=float, default=1.0)
    ap.add_argument("--body-linear-decay", type=float, default=0.0)
    ap.add_argument("--body-angular-decay", type=float, default=0.0)
    ap.add_argument("--body-drag-scale-x", type=float, default=1.0)
    ap.add_argument("--body-drag-scale-y", type=float, default=1.0)
    ap.add_argument("--body-drag-scale-z", type=float, default=1.0)
    ap.add_argument("--body-angular-damping-x", type=float, default=0.0)
    ap.add_argument("--body-angular-damping-y", type=float, default=0.0)
    ap.add_argument("--body-angular-damping-z", type=float, default=0.0)
    ap.add_argument("--motor-balance-x", type=float, default=0.0)
    ap.add_argument("--motor-balance-y", type=float, default=0.0)
    ap.add_argument("--moment-balance-x", type=float, default=0.0)
    ap.add_argument("--moment-balance-y", type=float, default=0.0)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    base_path = Path(args.base_candidate).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    base_payload = _load_candidate(base_path / "identified_parameters.json")
    variant = build_variant(
        base_payload=base_payload,
        geometry_preset=args.geometry_preset,
        time_constant_up_scale=args.time_constant_up_scale,
        time_constant_down_scale=args.time_constant_down_scale,
        motor_constant_scale=args.motor_constant_scale,
        moment_constant_scale=args.moment_constant_scale,
        rotor_drag_scale=args.rotor_drag_scale,
        rolling_moment_scale=args.rolling_moment_scale,
        slowdown_scale=args.slowdown_scale,
        body_linear_decay=args.body_linear_decay,
        body_angular_decay=args.body_angular_decay,
        body_drag_scale_x=args.body_drag_scale_x,
        body_drag_scale_y=args.body_drag_scale_y,
        body_drag_scale_z=args.body_drag_scale_z,
        body_angular_damping_x=args.body_angular_damping_x,
        body_angular_damping_y=args.body_angular_damping_y,
        body_angular_damping_z=args.body_angular_damping_z,
        motor_balance_x=args.motor_balance_x,
        motor_balance_y=args.motor_balance_y,
        moment_balance_x=args.moment_balance_x,
        moment_balance_y=args.moment_balance_y,
    )
    target = out_dir / "identified_parameters.json"
    target.write_text(json.dumps(variant, indent=2), encoding="utf-8")
    print(json.dumps({"ok": True, "out_dir": str(out_dir), "target": str(target)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
