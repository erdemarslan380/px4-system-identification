"""Shared scoring utilities for digital-twin validation."""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Mapping

from experimental_validation.reference_models import default_x500_reference

METRIC_SPECS: dict[str, dict[str, float | str]] = {
    "mass_kg": {"family": "mass", "weight": 1.0, "tolerance_pct": 5.0},
    "ixx_kgm2": {"family": "inertia", "weight": 1.4, "tolerance_pct": 10.0},
    "iyy_kgm2": {"family": "inertia", "weight": 1.2, "tolerance_pct": 10.0},
    "izz_kgm2": {"family": "inertia", "weight": 1.0, "tolerance_pct": 10.0},
    "time_constant_up_s": {"family": "motor_dynamics", "weight": 1.3, "tolerance_pct": 15.0},
    "time_constant_down_s": {"family": "motor_dynamics", "weight": 1.0, "tolerance_pct": 15.0},
    "max_rot_velocity_radps": {"family": "motor_dynamics", "weight": 0.8, "tolerance_pct": 5.0},
    "motor_constant": {"family": "motor_coefficients", "weight": 1.1, "tolerance_pct": 5.0},
    "moment_constant": {"family": "motor_coefficients", "weight": 0.9, "tolerance_pct": 5.0},
    "rotor_drag_coefficient": {"family": "motor_coefficients", "weight": 0.8, "tolerance_pct": 8.0},
    "rolling_moment_coefficient": {"family": "motor_coefficients", "weight": 0.6, "tolerance_pct": 8.0},
    "rotor_velocity_slowdown_sim": {"family": "motor_coefficients", "weight": 0.7, "tolerance_pct": 5.0},
}


def flatten_reference_metrics(reference: Mapping[str, Any] | None = None) -> dict[str, float]:
    reference = default_x500_reference() if reference is None else deepcopy(reference)
    return {
        "mass_kg": float(reference["mass"]["mass_kg"]),
        "ixx_kgm2": float(reference["inertia"]["x"]["inertia_kgm2"]),
        "iyy_kgm2": float(reference["inertia"]["y"]["inertia_kgm2"]),
        "izz_kgm2": float(reference["inertia"]["z"]["inertia_kgm2"]),
        "time_constant_up_s": float(reference["motor_model"]["time_constant_up_s"]),
        "time_constant_down_s": float(reference["motor_model"]["time_constant_down_s"]),
        "max_rot_velocity_radps": float(reference["motor_model"]["max_rot_velocity_radps"]),
        "motor_constant": float(reference["motor_model"]["motor_constant"]),
        "moment_constant": float(reference["motor_model"]["moment_constant"]),
        "rotor_drag_coefficient": float(reference["motor_model"]["rotor_drag_coefficient"]),
        "rolling_moment_coefficient": float(reference["motor_model"]["rolling_moment_coefficient"]),
        "rotor_velocity_slowdown_sim": float(reference["motor_model"]["rotor_velocity_slowdown_sim"]),
    }


def flatten_identified_metrics(identified: Mapping[str, Any]) -> dict[str, float]:
    return {
        "mass_kg": float(identified["mass"]["mass_kg"]),
        "ixx_kgm2": float(identified["inertia"]["x"]["inertia_kgm2"]),
        "iyy_kgm2": float(identified["inertia"]["y"]["inertia_kgm2"]),
        "izz_kgm2": float(identified["inertia"]["z"]["inertia_kgm2"]),
        "time_constant_up_s": float(identified["motor_model"]["time_constant_up_s"]["value"]),
        "time_constant_down_s": float(identified["motor_model"]["time_constant_down_s"]["value"]),
        "max_rot_velocity_radps": float(identified["motor_model"]["max_rot_velocity_radps"]["value"]),
        "motor_constant": float(identified["motor_model"]["motor_constant"]["value"]),
        "moment_constant": float(identified["motor_model"]["moment_constant"]["value"]),
        "rotor_drag_coefficient": float(identified["motor_model"]["rotor_drag_coefficient"]["value"]),
        "rolling_moment_coefficient": float(identified["motor_model"]["rolling_moment_coefficient"]["value"]),
        "rotor_velocity_slowdown_sim": float(identified["motor_model"]["rotor_velocity_slowdown_sim"]["value"]),
    }


def comparable_metrics_from_values(candidate: Mapping[str, float], reference: Mapping[str, float]) -> dict[str, dict[str, float]]:
    comparable: dict[str, dict[str, float]] = {}
    for metric_name in METRIC_SPECS:
        identified_value = float(candidate[metric_name])
        reference_value = float(reference[metric_name])
        abs_error = identified_value - reference_value
        pct_error = 0.0 if abs(reference_value) < 1e-12 else (abs_error / reference_value) * 100.0
        comparable[metric_name] = {
            "identified": identified_value,
            "reference": reference_value,
            "abs_error": abs_error,
            "pct_error": pct_error,
        }
    return comparable


def build_blended_twin_score(comparable_metrics: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    weighted_penalty = 0.0
    total_weight = 0.0
    family_penalty: dict[str, float] = {}
    family_weight: dict[str, float] = {}
    metric_contributions: dict[str, dict[str, float | str]] = {}

    for metric_name, metric in comparable_metrics.items():
        spec = METRIC_SPECS.get(metric_name)
        if spec is None:
            continue
        family = str(spec["family"])
        weight = float(spec["weight"])
        tolerance_pct = float(spec["tolerance_pct"])
        pct_error = float(metric["pct_error"])
        normalized_error = abs(pct_error) / tolerance_pct
        contribution = weight * normalized_error
        weighted_penalty += contribution
        total_weight += weight
        family_penalty[family] = family_penalty.get(family, 0.0) + contribution
        family_weight[family] = family_weight.get(family, 0.0) + weight
        metric_contributions[metric_name] = {
            "family": family,
            "weight": weight,
            "tolerance_pct": tolerance_pct,
            "normalized_error": normalized_error,
            "weighted_penalty": contribution,
        }

    normalized_penalty = 0.0 if total_weight <= 0.0 else weighted_penalty / total_weight
    score = 100.0 * math.exp(-normalized_penalty)

    family_scores = {
        family: 100.0 * math.exp(-(family_penalty[family] / family_weight[family]))
        for family in sorted(family_penalty)
    }

    return {
        "score": score,
        "normalized_penalty": normalized_penalty,
        "family_scores": family_scores,
        "metric_contributions": metric_contributions,
    }


def build_blended_twin_score_from_values(candidate: Mapping[str, float], reference: Mapping[str, float]) -> dict[str, Any]:
    comparable = comparable_metrics_from_values(candidate, reference)
    payload = build_blended_twin_score(comparable)
    payload["comparable_metrics"] = comparable
    return payload
