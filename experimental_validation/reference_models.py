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


# Legacy best-known family composite retained for regression comparisons.
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


# Current best practical candidate assembled from the historical x500 output pool.
# It keeps the strong inertia family from the digital-twin run, the slightly better
# mass estimate from composite_candidate_v5, and the stable motor family from
# composite_candidate_v2.
X500_FAMILY_COMPOSITE_V2: dict[str, Any] = {
    "mass": {
        "mass_kg": 2.064577114803053,
        "sample_count": 16564,
        "std_kg": 0.10567937756413882,
        "gravity_mps2": 9.80665,
    },
    "thrust_scale": {
        "thrust_scale_n_per_cmd": 33.0,
        "sample_count": 16564,
        "rmse_n": 0.0,
        "gravity_mps2": 9.80665,
    },
    "inertia": {
        "x": {"axis": "x", "inertia_kgm2": 0.026966278759017904, "sample_count": 6637, "rmse_nm": 0.09625700433345348},
        "y": {"axis": "y", "inertia_kgm2": 0.021553735357086597, "sample_count": 7671, "rmse_nm": 0.010405979272895261},
        "z": {"axis": "z", "inertia_kgm2": 0.04104662061456501, "sample_count": 6384, "rmse_nm": 0.06968089539146062},
    },
    "drag": {
        "x": {"axis": "x", "coefficient": -0.1235880107620473, "sample_count": 5329, "rmse_n": 0.4986161670740464},
        "y": {"axis": "y", "coefficient": -0.14283273062167107, "sample_count": 5352, "rmse_n": 0.4925913857257291},
        "z": {"axis": "z", "coefficient": -0.1091416401001524, "sample_count": 4623, "rmse_n": 0.2984600237268947},
    },
    "motor_model": {
        "time_constant_up_s": {"value": 0.011180450298120172, "sample_count": 171, "rmse": 0.0008960851278294116},
        "time_constant_down_s": {"value": 0.024999988507659682, "sample_count": 248, "rmse": 0.001643811943485457},
        "max_rot_velocity_radps": {"value": 1000.0, "sample_count": 63796, "rmse": 0.0},
        "motor_constant": {"value": 8.548580007960812e-06, "sample_count": 189432, "rmse": 2.7704455316293753e-07},
        "moment_constant": {"value": 0.01587158172248679, "sample_count": 46899, "rmse": 0.0016105189801886393},
        "rotor_drag_coefficient": {"value": 8.064279304826992e-05, "sample_count": 85701, "rmse": 2.5886056875627567e-07},
        "rolling_moment_coefficient": {"value": 1.0000030520838173e-06, "sample_count": 85701, "rmse": 2.599595117687427e-07},
        "rotor_velocity_slowdown_sim": {"value": 10.00000000037488, "sample_count": 189432, "rmse": 6.839121417355038e-08},
    },
    "warnings": [
        "Built-in example candidate based on the current best practical x500 family composite. Refresh this file once new SITL or real-flight identification outputs are available."
    ],
    "composite_sources": {
        "mass": "historical:x500_composite_candidate_v5",
        "inertia": "historical:x500_identification_digital_twin",
        "drag": "historical:x500_family_composite_v1",
        "motor_model": "historical:x500_composite_candidate_v2",
    },
}


def default_x500_reference() -> dict[str, Any]:
    return deepcopy(X500_REFERENCE_SDF)


def default_candidate_identified() -> dict[str, Any]:
    return deepcopy(X500_FAMILY_COMPOSITE_V2)
