#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.collect_sitl_tracking_dataset import collect_tracking_dataset
from experimental_validation.materialize_bridge_candidate_variant import build_variant
from experimental_validation.prepare_identified_model import prepare_identified_model
from experimental_validation.reference_models import default_jmavsim_prior_candidate
from experimental_validation.run_sitl_repeatability_matrix import build_mean_tracking_csv
from experimental_validation.run_sitl_validation import ValidationModelSpec
from experimental_validation.trajectory_catalog import validation_trajectory_id_map
from experimental_validation.trajectory_comparison_figures import (
    _interp_curve_at_progress,
    _load_tracking,
    _normalized_arclength,
    _trim_dataset,
)


MODEL_NAME = "x500_bridge_calibrated"
MODEL_SPEC = ValidationModelSpec("bridge_calibration", MODEL_NAME, "Bridge calibration candidate", truth_state_bridge=False)
QUERY = np.linspace(0.0, 1.0, 700)


STAGE1_GEOMETRIES = (
    {"name": "geom_x500_scaled", "geometry_preset": "x500_scaled_165mm"},
    {"name": "geom_jmavsim_keep_spin", "geometry_preset": "jmavsim_direct_keep_spin"},
    {"name": "geom_jmavsim_swap_spin", "geometry_preset": "jmavsim_direct_swap_spin"},
)

def _stage2_preset(
    name: str,
    *,
    time_constant_up_scale: float = 1.0,
    time_constant_down_scale: float = 1.0,
    motor_constant_scale: float = 1.0,
    moment_constant_scale: float = 1.0,
    rotor_drag_scale: float = 1.0,
    rolling_moment_scale: float = 1.0,
    slowdown_scale: float = 1.0,
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
) -> dict[str, float | str]:
    return {
        "name": name,
        "time_constant_up_scale": time_constant_up_scale,
        "time_constant_down_scale": time_constant_down_scale,
        "motor_constant_scale": motor_constant_scale,
        "moment_constant_scale": moment_constant_scale,
        "rotor_drag_scale": rotor_drag_scale,
        "rolling_moment_scale": rolling_moment_scale,
        "slowdown_scale": slowdown_scale,
        "body_linear_decay": body_linear_decay,
        "body_angular_decay": body_angular_decay,
        "body_drag_scale_x": body_drag_scale_x,
        "body_drag_scale_y": body_drag_scale_y,
        "body_drag_scale_z": body_drag_scale_z,
        "body_angular_damping_x": body_angular_damping_x,
        "body_angular_damping_y": body_angular_damping_y,
        "body_angular_damping_z": body_angular_damping_z,
        "motor_balance_x": motor_balance_x,
        "motor_balance_y": motor_balance_y,
        "moment_balance_x": moment_balance_x,
        "moment_balance_y": moment_balance_y,
    }


STAGE2_PRESETS = (
    _stage2_preset("base"),
    _stage2_preset(
        "thrust_up",
        motor_constant_scale=1.15,
        moment_constant_scale=1.10,
        rotor_drag_scale=0.75,
        rolling_moment_scale=0.90,
        slowdown_scale=0.90,
    ),
    _stage2_preset(
        "thrust_fast",
        time_constant_up_scale=0.82,
        time_constant_down_scale=0.82,
        motor_constant_scale=1.15,
        moment_constant_scale=1.10,
        rotor_drag_scale=0.75,
        rolling_moment_scale=0.90,
        slowdown_scale=0.90,
    ),
    _stage2_preset(
        "thrust_fast_yaw",
        time_constant_up_scale=0.82,
        time_constant_down_scale=0.82,
        motor_constant_scale=1.12,
        moment_constant_scale=1.20,
        rotor_drag_scale=0.70,
        rolling_moment_scale=0.85,
        slowdown_scale=0.85,
    ),
    _stage2_preset("dragx_low", body_drag_scale_x=0.5),
    _stage2_preset("dragx_high", body_drag_scale_x=1.6),
    _stage2_preset("dragy_low", body_drag_scale_y=0.5),
    _stage2_preset("dragy_high", body_drag_scale_y=1.6),
    _stage2_preset("dragxy_low", body_drag_scale_x=0.6, body_drag_scale_y=0.6),
    _stage2_preset("dragxy_high", body_drag_scale_x=1.5, body_drag_scale_y=1.5),
    _stage2_preset("dragz_low", body_drag_scale_z=0.6),
    _stage2_preset("dragz_high", body_drag_scale_z=1.5),
    _stage2_preset("yawdamp_002", body_angular_damping_z=0.02),
    _stage2_preset("yawdamp_005", body_angular_damping_z=0.05),
    _stage2_preset("yaw_soft_dragy_high", moment_constant_scale=0.8, body_drag_scale_y=1.5),
    _stage2_preset("yaw_strong_dragy_low", moment_constant_scale=1.2, body_drag_scale_y=0.6),
    _stage2_preset(
        "fast_dragxy_high",
        time_constant_up_scale=0.88,
        time_constant_down_scale=0.88,
        body_drag_scale_x=1.4,
        body_drag_scale_y=1.4,
    ),
    _stage2_preset(
        "slow_dragxy_low",
        time_constant_up_scale=1.15,
        time_constant_down_scale=1.15,
        body_drag_scale_x=0.7,
        body_drag_scale_y=0.7,
    ),
)


def _bridge_probe_seed(base_payload: dict) -> dict[str, float]:
    bridge_probe = (
        base_payload.get("indirect_observables", {})
        .get("bridge_probe", {})
    )

    def _metric(axis: str, key: str) -> float:
        try:
            return float(bridge_probe[axis][key]["value"])
        except Exception:
            return 0.0

    x_pos = _metric("x", "accel_scale_positive")
    x_neg = _metric("x", "accel_scale_negative")
    y_pos = _metric("y", "accel_scale_positive")
    y_neg = _metric("y", "accel_scale_negative")

    seed_x = 0.5 * (x_pos - x_neg)
    seed_y = 0.5 * (y_pos - y_neg)
    seed_x = float(np.clip(seed_x, -0.12, 0.12))
    seed_y = float(np.clip(seed_y, -0.12, 0.12))
    return {"x": seed_x, "y": seed_y}


def _bridge_probe_response_seed(base_payload: dict) -> dict[str, float]:
    bridge_probe = (
        base_payload.get("indirect_observables", {})
        .get("bridge_probe", {})
    )

    def _metric(axis: str, key: str, default: float = 1.0) -> float:
        try:
            value = float(bridge_probe[axis][key]["value"])
            if np.isfinite(value) and abs(value) > 1e-6:
                return value
        except Exception:
            pass
        return default

    x_velocity_scale = abs(_metric("x", "velocity_scale", 1.0))
    y_velocity_scale = abs(_metric("y", "velocity_scale", 1.0))
    x_accel_scale = abs(_metric("x", "accel_scale", 1.0))
    y_accel_scale = abs(_metric("y", "accel_scale", 1.0))

    motor_constant_scale = float(np.clip(0.5 * (x_accel_scale + y_accel_scale), 0.85, 1.30))
    body_drag_scale_x = float(np.clip(1.0 / max(0.60, x_velocity_scale), 0.60, 1.50))
    body_drag_scale_y = float(np.clip(1.0 / max(0.60, y_velocity_scale), 0.60, 1.50))
    return {
        "motor_constant_scale": motor_constant_scale,
        "body_drag_scale_x": body_drag_scale_x,
        "body_drag_scale_y": body_drag_scale_y,
    }


def _state_chain_seed(base_payload: dict) -> dict[str, float]:
    bridge_probe = (
        base_payload.get("state_chain", {})
        .get("bridge_probe_xy", {})
    )

    def _step_metric(axis: str, branch: str, node: str) -> float:
        try:
            value = float(bridge_probe[axis][branch][node]["value"])
            if np.isfinite(value) and value > 0.0:
                return value
        except Exception:
            pass
        return 0.0

    delays = [
        _step_metric("x", "up", "delay_s"),
        _step_metric("x", "down", "delay_s"),
        _step_metric("y", "up", "delay_s"),
        _step_metric("y", "down", "delay_s"),
    ]
    taus = [
        _step_metric("x", "up", "time_constant_s"),
        _step_metric("x", "down", "time_constant_s"),
        _step_metric("y", "up", "time_constant_s"),
        _step_metric("y", "down", "time_constant_s"),
    ]
    delays = [value for value in delays if value > 0.0]
    taus = [value for value in taus if value > 0.0]
    if not delays:
        for axis in ("x", "y"):
            try:
                value = float(bridge_probe[axis]["accel_correlation_lag_s"]["value"])
                if np.isfinite(value) and value > 0.0:
                    delays.append(value)
            except Exception:
                pass
    mean_delay = float(np.mean(delays)) if delays else 0.0
    mean_tau = float(np.mean(taus)) if taus else 0.0
    time_scale = float(np.clip(1.0 + (mean_delay + mean_tau) / 0.25, 0.90, 1.35))
    return {
        "mean_delay_s": mean_delay,
        "mean_tau_s": mean_tau,
        "time_scale": time_scale,
    }


def _control_effort_seed(base_payload: dict) -> dict[str, float]:
    control_effort = base_payload.get("control_effort", {})

    def _metric(path: tuple[str, ...], default: float = 0.0) -> float:
        node = control_effort
        try:
            for key in path:
                node = node[key]
            value = float(node["value"])
            if np.isfinite(value):
                return value
        except Exception:
            pass
        return default

    yaw_unalloc = _metric(("attitude_sweeps", "yaw", "mean_abs_alloc_unalloc_yaw"), 0.0)
    yaw_moment_scale = float(np.clip(1.0 + yaw_unalloc * 2.0, 0.85, 1.20))
    return {
        "yaw_moment_scale": yaw_moment_scale,
    }


def _build_stage2_presets(base_payload: dict) -> tuple[dict[str, float | str], ...]:
    probe_seed = _bridge_probe_seed(base_payload)
    response_seed = _bridge_probe_response_seed(base_payload)
    state_seed = _state_chain_seed(base_payload)
    effort_seed = _control_effort_seed(base_payload)
    seed_x = probe_seed["x"]
    seed_y = probe_seed["y"]
    dynamic = []
    if abs(seed_x) > 1e-4 or abs(seed_y) > 1e-4:
        dynamic.extend(
            [
                _stage2_preset(
                    "thrust_up_probe_motor",
                    motor_constant_scale=1.15,
                    moment_constant_scale=1.10,
                    rotor_drag_scale=0.75,
                    rolling_moment_scale=0.90,
                    slowdown_scale=0.90,
                    motor_balance_x=seed_x,
                    motor_balance_y=seed_y,
                ),
                _stage2_preset(
                    "thrust_up_probe_motor_inv",
                    motor_constant_scale=1.15,
                    moment_constant_scale=1.10,
                    rotor_drag_scale=0.75,
                    rolling_moment_scale=0.90,
                    slowdown_scale=0.90,
                    motor_balance_x=-seed_x,
                    motor_balance_y=-seed_y,
                ),
                _stage2_preset(
                    "thrust_up_probe_full",
                    motor_constant_scale=1.15,
                    moment_constant_scale=1.10,
                    rotor_drag_scale=0.75,
                    rolling_moment_scale=0.90,
                    slowdown_scale=0.90,
                    motor_balance_x=seed_x,
                    motor_balance_y=seed_y,
                    moment_balance_x=seed_x,
                    moment_balance_y=seed_y,
                ),
                _stage2_preset(
                    "thrust_up_probe_full_inv",
                    motor_constant_scale=1.15,
                    moment_constant_scale=1.10,
                    rotor_drag_scale=0.75,
                    rolling_moment_scale=0.90,
                    slowdown_scale=0.90,
                    motor_balance_x=-seed_x,
                    motor_balance_y=-seed_y,
                    moment_balance_x=-seed_x,
                    moment_balance_y=-seed_y,
                ),
            ]
        )
    dynamic.extend(
        [
            _stage2_preset(
                "probe_response_seed",
                time_constant_up_scale=state_seed["time_scale"],
                time_constant_down_scale=state_seed["time_scale"],
                motor_constant_scale=response_seed["motor_constant_scale"],
                moment_constant_scale=effort_seed["yaw_moment_scale"],
                rotor_drag_scale=0.75,
                rolling_moment_scale=0.90,
                slowdown_scale=0.90,
                body_drag_scale_x=response_seed["body_drag_scale_x"],
                body_drag_scale_y=response_seed["body_drag_scale_y"],
                motor_balance_x=seed_x,
                motor_balance_y=seed_y,
                moment_balance_x=seed_x,
                moment_balance_y=seed_y,
            ),
            _stage2_preset(
                "probe_response_seed_slow",
                time_constant_up_scale=float(np.clip(state_seed["time_scale"] * 1.12, 0.90, 1.45)),
                time_constant_down_scale=float(np.clip(state_seed["time_scale"] * 1.12, 0.90, 1.45)),
                motor_constant_scale=response_seed["motor_constant_scale"],
                moment_constant_scale=effort_seed["yaw_moment_scale"],
                rotor_drag_scale=0.75,
                rolling_moment_scale=0.90,
                slowdown_scale=0.90,
                body_drag_scale_x=response_seed["body_drag_scale_x"],
                body_drag_scale_y=response_seed["body_drag_scale_y"],
                motor_balance_x=seed_x,
                motor_balance_y=seed_y,
                moment_balance_x=seed_x,
                moment_balance_y=seed_y,
            ),
            _stage2_preset(
                "probe_response_seed_fast",
                time_constant_up_scale=float(np.clip(state_seed["time_scale"] * 0.88, 0.75, 1.30)),
                time_constant_down_scale=float(np.clip(state_seed["time_scale"] * 0.88, 0.75, 1.30)),
                motor_constant_scale=response_seed["motor_constant_scale"],
                moment_constant_scale=effort_seed["yaw_moment_scale"],
                rotor_drag_scale=0.75,
                rolling_moment_scale=0.90,
                slowdown_scale=0.90,
                body_drag_scale_x=response_seed["body_drag_scale_x"],
                body_drag_scale_y=response_seed["body_drag_scale_y"],
                motor_balance_x=seed_x,
                motor_balance_y=seed_y,
                moment_balance_x=seed_x,
                moment_balance_y=seed_y,
            ),
        ]
    )
    return tuple(STAGE2_PRESETS) + tuple(dynamic)


def _node_value(payload: dict, key: str) -> float:
    try:
        return float(payload["motor_model"][key]["value"])
    except Exception:
        return 0.0


def _complete_missing_motor_model_from_jmavsim_prior(base_payload: dict) -> dict:
    payload = deepcopy(base_payload)
    prior = default_jmavsim_prior_candidate()
    warnings = payload.setdefault("warnings", [])
    motor_model = payload.setdefault("motor_model", {})
    filled: list[str] = []

    for key, prior_node in prior["motor_model"].items():
        current = _node_value(payload, key)
        if not np.isfinite(current) or current <= 0.0:
            motor_model[key] = deepcopy(prior_node)
            filled.append(key)

    if filled:
        warnings.append(
            "Bridge calibration filled missing motor-model fields from jMAVSim prior: "
            + ", ".join(sorted(filled))
        )
        payload.setdefault("composite_sources", {})["motor_model_bridge_seed"] = "jmavsim_prior"

    return payload


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Calibrate a SITL bridge candidate against HIL calibration trajectories.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--base-candidate-dir", required=True)
    ap.add_argument("--case-name", default="lemniscate")
    ap.add_argument("--hil-calibration-csv", nargs="+", required=True)
    ap.add_argument("--out-root", required=True)
    ap.add_argument(
        "--geometry-preset-only",
        default="",
        help="Skip stage-1 geometry search and force a single geometry preset (for example: x500_scaled_165mm).",
    )
    return ap.parse_args()


def _ensure_clean_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def _load_pos_curve(case_name: str, csv_path: Path) -> np.ndarray:
    timestamps_s, ref, pos = _load_tracking(csv_path)
    ref_trimmed, pos_trimmed = _trim_dataset(case_name, timestamps_s, ref, pos)
    progress = _normalized_arclength(ref_trimmed)
    keep = np.concatenate(([True], np.diff(progress) > 1e-9))
    return _interp_curve_at_progress(pos_trimmed[keep], QUERY)


def _distance_metrics(case_name: str, candidate_csv: Path, target_mean_csv: Path) -> dict[str, float]:
    cand = _load_pos_curve(case_name, candidate_csv)
    tgt = _load_pos_curve(case_name, target_mean_csv)
    delta = cand - tgt
    err = np.linalg.norm(delta, axis=1)
    xy = np.linalg.norm(delta[:, :2], axis=1)
    z = np.abs(delta[:, 2])
    return {
        "rmse_to_hil_mean_m": float(np.sqrt(np.mean(err ** 2))),
        "mean_to_hil_mean_m": float(np.mean(err)),
        "max_to_hil_mean_m": float(np.max(err)),
        "xy_rmse_to_hil_mean_m": float(np.sqrt(np.mean(xy ** 2))),
        "z_rmse_to_hil_mean_m": float(np.sqrt(np.mean(z ** 2))),
    }


def _write_candidate(out_dir: Path, payload: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / "identified_parameters.json"
    target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return target


def _evaluate_candidate(
    *,
    px4_root: Path,
    case_name: str,
    traj_id: int,
    base_payload: dict,
    variant_name: str,
    variant_kwargs: dict,
    out_root: Path,
    hil_mean_csv: Path,
) -> dict:
    candidate_dir = out_root / "candidates" / variant_name
    runtime_root = out_root / "runs" / variant_name
    _ensure_clean_dir(candidate_dir)
    _ensure_clean_dir(runtime_root)

    payload = build_variant(base_payload=base_payload, **variant_kwargs)
    _write_candidate(candidate_dir, payload)
    prepare_identified_model(px4_root, candidate_dir, model_name=MODEL_NAME)

    entry = validation_trajectory_id_map()[traj_id]
    dataset = collect_tracking_dataset(
        px4_root=px4_root,
        out_root=runtime_root,
        candidate_dir=candidate_dir,
        model_spec=MODEL_SPEC,
        trajectories=(entry,),
        headless=True,
        show_console=False,
        skip_landing_after_trajectories=True,
    )
    case = dataset["cases"][0]
    result = {
        "variant_name": variant_name,
        "variant_kwargs": variant_kwargs,
        "candidate_dir": str(candidate_dir.resolve()),
        "ok": bool(case.get("tracking_log")),
        "case_error": case.get("error"),
        "tracking_log": case.get("tracking_log"),
    }
    if case.get("tracking_log"):
        metrics = _distance_metrics(case_name, Path(str(case["tracking_log"])), hil_mean_csv)
        result.update(metrics)
    return result


def main() -> int:
    args = parse_args()
    px4_root = Path(args.px4_root).expanduser().resolve()
    base_candidate_dir = Path(args.base_candidate_dir).expanduser().resolve()
    out_root = Path(args.out_root).expanduser().resolve()
    case_name = args.case_name
    traj_id = validation_trajectory_id_map()[next(k for k, v in validation_trajectory_id_map().items() if v.name == case_name)].traj_id
    hil_csvs = [Path(item).expanduser().resolve() for item in args.hil_calibration_csv]

    _ensure_clean_dir(out_root)
    hil_mean_root = out_root / "hil_calibration"
    hil_mean_root.mkdir(parents=True, exist_ok=True)
    hil_mean_csv = hil_mean_root / f"{case_name}_hil_mean.csv"
    hil_mean = build_mean_tracking_csv(case_name, hil_csvs, hil_mean_csv, samples=700)

    base_payload = json.loads((base_candidate_dir / "identified_parameters.json").read_text(encoding="utf-8"))
    base_payload = _complete_missing_motor_model_from_jmavsim_prior(base_payload)

    stage1_results = []
    forced_geometry = args.geometry_preset_only.strip()
    if forced_geometry:
        best_stage1 = {
            "variant_name": f"forced_{forced_geometry}",
            "variant_kwargs": {
                "geometry_preset": forced_geometry,
                "time_constant_up_scale": 1.0,
                "time_constant_down_scale": 1.0,
                "motor_constant_scale": 1.0,
                "moment_constant_scale": 1.0,
                "rotor_drag_scale": 1.0,
                "rolling_moment_scale": 1.0,
                "slowdown_scale": 1.0,
                "body_linear_decay": 0.0,
                "body_angular_decay": 0.0,
                "body_drag_scale_x": 1.0,
                "body_drag_scale_y": 1.0,
                "body_drag_scale_z": 1.0,
                "body_angular_damping_x": 0.0,
                "body_angular_damping_y": 0.0,
                "body_angular_damping_z": 0.0,
                "motor_balance_x": 0.0,
                "motor_balance_y": 0.0,
                "moment_balance_x": 0.0,
                "moment_balance_y": 0.0,
            },
            "ok": True,
        }
    else:
        for preset in STAGE1_GEOMETRIES:
            kwargs = {
                "geometry_preset": preset["geometry_preset"],
                "time_constant_up_scale": 1.0,
                "time_constant_down_scale": 1.0,
                "motor_constant_scale": 1.0,
                "moment_constant_scale": 1.0,
                "rotor_drag_scale": 1.0,
                "rolling_moment_scale": 1.0,
                "slowdown_scale": 1.0,
                "body_linear_decay": 0.0,
                "body_angular_decay": 0.0,
                "body_drag_scale_x": 1.0,
                "body_drag_scale_y": 1.0,
                "body_drag_scale_z": 1.0,
                "body_angular_damping_x": 0.0,
                "body_angular_damping_y": 0.0,
                "body_angular_damping_z": 0.0,
                "motor_balance_x": 0.0,
                "motor_balance_y": 0.0,
                "moment_balance_x": 0.0,
                "moment_balance_y": 0.0,
            }
            stage1_results.append(
                _evaluate_candidate(
                    px4_root=px4_root,
                    case_name=case_name,
                    traj_id=traj_id,
                    base_payload=base_payload,
                    variant_name=preset["name"],
                    variant_kwargs=kwargs,
                    out_root=out_root / "stage1",
                    hil_mean_csv=hil_mean_csv,
                )
            )

        successful_stage1 = [item for item in stage1_results if item.get("ok")]
        if not successful_stage1:
            raise RuntimeError("No successful stage-1 calibration candidates")
        best_stage1 = min(successful_stage1, key=lambda item: float(item["rmse_to_hil_mean_m"]))

    stage2_results = []
    best_geometry = best_stage1["variant_kwargs"]["geometry_preset"]
    for preset in _build_stage2_presets(base_payload):
        kwargs = {
            "geometry_preset": best_geometry,
            "time_constant_up_scale": preset["time_constant_up_scale"],
            "time_constant_down_scale": preset["time_constant_down_scale"],
            "motor_constant_scale": preset["motor_constant_scale"],
            "moment_constant_scale": preset["moment_constant_scale"],
            "rotor_drag_scale": preset["rotor_drag_scale"],
            "rolling_moment_scale": preset["rolling_moment_scale"],
            "slowdown_scale": preset["slowdown_scale"],
            "body_linear_decay": preset["body_linear_decay"],
            "body_angular_decay": preset["body_angular_decay"],
            "body_drag_scale_x": preset["body_drag_scale_x"],
            "body_drag_scale_y": preset["body_drag_scale_y"],
            "body_drag_scale_z": preset["body_drag_scale_z"],
            "body_angular_damping_x": preset["body_angular_damping_x"],
            "body_angular_damping_y": preset["body_angular_damping_y"],
            "body_angular_damping_z": preset["body_angular_damping_z"],
            "motor_balance_x": preset["motor_balance_x"],
            "motor_balance_y": preset["motor_balance_y"],
            "moment_balance_x": preset["moment_balance_x"],
            "moment_balance_y": preset["moment_balance_y"],
        }
        stage2_results.append(
            _evaluate_candidate(
                px4_root=px4_root,
                case_name=case_name,
                traj_id=traj_id,
                base_payload=base_payload,
                variant_name=f"{best_geometry}__{preset['name']}",
                variant_kwargs=kwargs,
                out_root=out_root / "stage2",
                hil_mean_csv=hil_mean_csv,
            )
        )

    successful_stage2 = [item for item in stage2_results if item.get("ok")]
    if not successful_stage2:
        raise RuntimeError("No successful stage-2 calibration candidates")
    best = min(successful_stage2, key=lambda item: float(item["rmse_to_hil_mean_m"]))

    best_final_dir = out_root / "best_candidate"
    _ensure_clean_dir(best_final_dir)
    best_payload = build_variant(base_payload=base_payload, **best["variant_kwargs"])
    _write_candidate(best_final_dir, best_payload)

    summary = {
        "case_name": case_name,
        "hil_calibration_csv": [str(path) for path in hil_csvs],
        "hil_mean_csv": str(hil_mean_csv.resolve()),
        "hil_mean_summary": hil_mean,
        "stage1": stage1_results,
        "stage2": stage2_results,
        "best": best,
        "best_candidate_dir": str(best_final_dir.resolve()),
    }
    (out_root / "calibration_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({"ok": True, "best_candidate_dir": summary["best_candidate_dir"], "summary": str((out_root / 'calibration_summary.json').resolve())}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
