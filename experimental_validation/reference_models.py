"""Reference and example candidate parameter sets used by the validation tools."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

X500_REFERENCE_SDF: dict[str, Any] = {
    "model_name": "x500",
    "model_sdf": "builtin:x500",
    "base_model_sdf": "builtin:x500_base",
    "mass": {"mass_kg": 2.0},
    "inertia": {
        "x": {"inertia_kgm2": 0.02166666666666667},
        "y": {"inertia_kgm2": 0.02166666666666667},
        "z": {"inertia_kgm2": 0.04000000000000001},
    },
    "motor_model": {
        "time_constant_up_s": 0.0125,
        "time_constant_down_s": 0.025,
        "max_rot_velocity_radps": 1000.0,
        "motor_constant": 8.54858e-06,
        "moment_constant": 0.016,
        "rotor_drag_coefficient": 8.06428e-05,
        "rolling_moment_coefficient": 1.0e-06,
        "rotor_velocity_slowdown_sim": 10.0,
    },
}


# This example candidate mirrors the best-known family composite obtained so far.
X500_FAMILY_COMPOSITE_V1: dict[str, Any] = {
    "mass": {"mass_kg": 2.06636, "sample_count": 0},
    "thrust_scale": {"thrust_scale_n_per_cmd": 33.0, "sample_count": 0},
    "inertia": {
        "x": {"inertia_kgm2": 0.0269658, "sample_count": 0},
        "y": {"inertia_kgm2": 0.0215538, "sample_count": 0},
        "z": {"inertia_kgm2": 0.0410468, "sample_count": 0},
    },
    "drag": {
        "x": {"coefficient": 0.1800, "sample_count": 0},
        "y": {"coefficient": 0.2100, "sample_count": 0},
        "z": {"coefficient": 0.2700, "sample_count": 0},
    },
    "motor_model": {
        "time_constant_up_s": {"value": 0.0111805, "sample_count": 0},
        "time_constant_down_s": {"value": 0.0250000, "sample_count": 0},
        "max_rot_velocity_radps": {"value": 1000.0, "sample_count": 0},
        "motor_constant": {"value": 8.54858e-06, "sample_count": 0},
        "moment_constant": {"value": 0.0158563, "sample_count": 0},
        "rotor_drag_coefficient": {"value": 8.06428e-05, "sample_count": 0},
        "rolling_moment_coefficient": {"value": 1.0e-06, "sample_count": 0},
        "rotor_velocity_slowdown_sim": {"value": 10.0, "sample_count": 0},
    },
    "warnings": [
        "Built-in example candidate based on the current best x500 family composite. Replace this file with a fresh identified_parameters.json once real-flight or new SITL identification outputs are available."
    ],
}


def default_x500_reference() -> dict[str, Any]:
    return deepcopy(X500_REFERENCE_SDF)


def default_candidate_identified() -> dict[str, Any]:
    return deepcopy(X500_FAMILY_COMPOSITE_V1)
