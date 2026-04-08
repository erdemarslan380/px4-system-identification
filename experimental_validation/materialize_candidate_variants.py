#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.reference_models import default_jmavsim_prior_candidate, default_x500_reference


def _candidate_from_reference(*, label: str, warnings: list[str] | None = None) -> dict:
    ref = default_x500_reference()
    motor = ref["motor_model"]
    max_total_thrust = 4.0 * float(motor["motor_constant"]) * float(motor["max_rot_velocity_radps"]) ** 2
    return {
        "mass": {
            "mass_kg": float(ref["mass"]["mass_kg"]),
            "sample_count": 0,
            "std_kg": 0.0,
            "gravity_mps2": 9.80665,
        },
        "thrust_scale": {
            "thrust_scale_n_per_cmd": max_total_thrust,
            "sample_count": 0,
            "rmse_n": 0.0,
            "gravity_mps2": 9.80665,
        },
        "inertia": {
            "x": {"axis": "x", "inertia_kgm2": float(ref["inertia"]["x"]["inertia_kgm2"]), "sample_count": 0, "rmse_nm": 0.0},
            "y": {"axis": "y", "inertia_kgm2": float(ref["inertia"]["y"]["inertia_kgm2"]), "sample_count": 0, "rmse_nm": 0.0},
            "z": {"axis": "z", "inertia_kgm2": float(ref["inertia"]["z"]["inertia_kgm2"]), "sample_count": 0, "rmse_nm": 0.0},
        },
        "drag": {
            "x": {"axis": "x", "coefficient": 0.0, "sample_count": 0, "rmse_n": 0.0},
            "y": {"axis": "y", "coefficient": 0.0, "sample_count": 0, "rmse_n": 0.0},
            "z": {"axis": "z", "coefficient": 0.0, "sample_count": 0, "rmse_n": 0.0},
        },
        "motor_model": {
            "time_constant_up_s": {"value": float(motor["time_constant_up_s"]), "sample_count": 0, "rmse": 0.0},
            "time_constant_down_s": {"value": float(motor["time_constant_down_s"]), "sample_count": 0, "rmse": 0.0},
            "max_rot_velocity_radps": {"value": float(motor["max_rot_velocity_radps"]), "sample_count": 0, "rmse": 0.0},
            "motor_constant": {"value": float(motor["motor_constant"]), "sample_count": 0, "rmse": 0.0},
            "moment_constant": {"value": float(motor["moment_constant"]), "sample_count": 0, "rmse": 0.0},
            "rotor_drag_coefficient": {"value": float(motor["rotor_drag_coefficient"]), "sample_count": 0, "rmse": 0.0},
            "rolling_moment_coefficient": {"value": float(motor["rolling_moment_coefficient"]), "sample_count": 0, "rmse": 0.0},
            "rotor_velocity_slowdown_sim": {"value": float(motor["rotor_velocity_slowdown_sim"]), "sample_count": 0, "rmse": 0.0},
        },
        "warnings": warnings or [label],
        "composite_sources": {
            "mass": "stock:x500_base",
            "inertia": "stock:x500_base",
            "drag": "manual:zero_placeholder",
            "motor_model": "stock:x500 motor plugins",
        },
    }


def _with_mass(candidate: dict, *, mass_kg: float, note: str) -> dict:
    out = deepcopy(candidate)
    out["mass"]["mass_kg"] = float(mass_kg)
    warnings = list(out.get("warnings") or [])
    warnings.append(note)
    out["warnings"] = warnings
    return out


def _write_candidate(out_dir: Path, payload: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / "identified_parameters.json"
    target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return target


def materialize_default_variants(*, repo_root: Path) -> dict[str, str]:
    jmavsim_prior = default_jmavsim_prior_candidate()
    stock_exact = _candidate_from_reference(label="Built from stock x500 SDF reference.")

    outputs: dict[str, str] = {}
    outputs["jmavsim_prior_plus100g_v1"] = str(
        _write_candidate(
            repo_root / "examples" / "paper_assets" / "candidates" / "jmavsim_prior_plus100g_v1",
            _with_mass(jmavsim_prior, mass_kg=0.9, note="Mass increased by +0.1 kg from jMAVSim prior."),
        ).resolve()
    )
    outputs["stock_sdf_exact_v1"] = str(
        _write_candidate(
            repo_root / "examples" / "paper_assets" / "candidates" / "stock_sdf_exact_v1",
            stock_exact,
        ).resolve()
    )
    outputs["stock_sdf_minus100g_v1"] = str(
        _write_candidate(
            repo_root / "examples" / "paper_assets" / "candidates" / "stock_sdf_minus100g_v1",
            _with_mass(stock_exact, mass_kg=float(stock_exact["mass"]["mass_kg"]) - 0.1, note="Mass reduced by -0.1 kg from stock x500 SDF."),
        ).resolve()
    )
    return outputs


def main() -> int:
    ap = argparse.ArgumentParser(description="Materialize persistent candidate JSON variants used by SITL studies.")
    ap.add_argument("--repo-root", default=str(REPO_ROOT))
    args = ap.parse_args()
    outputs = materialize_default_variants(repo_root=Path(args.repo_root).expanduser().resolve())
    print(json.dumps(outputs, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
