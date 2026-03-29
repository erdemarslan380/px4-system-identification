#!/usr/bin/env python3
"""Run a compact off-nominal SITL methodology study for x500.

Study flow:
1. Run the five validation trajectories with the stock x500 model and PX4 baseline control.
2. Create an off-nominal x500 SDF with small parameter perturbations.
3. Run the nine identification maneuvers on the off-nominal model.
4. Re-identify the model parameters from those CSV logs.
5. Run the five validation trajectories again on the off-nominal model in a light-wind world.
6. Generate two combined 3D figures:
   - circle + hairpin + lemniscate
   - time_optimal_30s + minimum_snap_50s
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Line3DCollection

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import parse_x500_sdf_reference
from experimental_validation.run_sitl_validation import (
    PX4_PROMPT,
    TRACKING_LOG_START_RE,
    Px4SitlSession,
    ValidationModelSpec,
    _build_px4_env,
    _cleanup_stale_sitl_processes,
    _copy_log,
    _find_tracking_log,
    _inject_truth_logger_plugin,
    _prepare_run_rootfs,
)
from experimental_validation.validation_trajectories import (
    DEFAULT_VALIDATION_TRAJECTORIES as VALIDATION_CASES,
    export_validation_trajectories,
)

IDENT_PROFILES: tuple[str, ...] = (
    "mass_vertical",
    "hover_thrust",
    "roll_sweep",
    "pitch_sweep",
    "yaw_sweep",
    "drag_x",
    "drag_y",
    "drag_z",
    "motor_step",
)

IDENT_LOG_START_RE = re.compile(r"Identification log started: (\./identification_logs/[^\r\n]+\.csv)")
IDENT_DONE_RE = re.compile(r"Identification maneuver completed: ([a-z_]+)")


@dataclass(frozen=True)
class OffnominalPerturbation:
    mass_scale: float = 0.95
    inertia_scale: float = 0.95
    time_constant_up_scale: float = 1.05
    time_constant_down_scale: float = 1.05
    max_rot_velocity_scale: float = 0.97
    motor_constant_scale: float = 1.03
    moment_constant_scale: float = 0.97
    rotor_drag_scale: float = 1.05
    rolling_moment_scale: float = 1.05


PANEL_GROUPS: tuple[tuple[str, ...], ...] = (
    ("circle", "hairpin", "lemniscate"),
    ("time_optimal_30s", "minimum_snap_50s"),
)

SITL_ESC_MIN = 0
SITL_ESC_MAX = 55


def _trajectory_duration_s(entry) -> float:
    value = getattr(entry, "nominal_duration_s", None)
    if value is None:
        value = getattr(entry, "duration_s")
    return float(value)


def _indent(elem: ET.Element, level: int = 0) -> None:
    text = "\n" + ("  " * level)
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = text + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = text
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = text


def _set_text(root: ET.Element, path: str, value: float | str) -> None:
    node = root.find(path)
    if node is None:
        raise KeyError(f"Missing XML path: {path}")
    node.text = str(value)


def offnominal_reference(reference: dict, perturbation: OffnominalPerturbation) -> dict[str, float]:
    return {
        "mass_kg": float(reference["mass"]["mass_kg"]) * perturbation.mass_scale,
        "ixx_kgm2": float(reference["inertia"]["x"]["inertia_kgm2"]) * perturbation.inertia_scale,
        "iyy_kgm2": float(reference["inertia"]["y"]["inertia_kgm2"]) * perturbation.inertia_scale,
        "izz_kgm2": float(reference["inertia"]["z"]["inertia_kgm2"]) * perturbation.inertia_scale,
        "time_constant_up_s": float(reference["motor_model"]["time_constant_up_s"]) * perturbation.time_constant_up_scale,
        "time_constant_down_s": float(reference["motor_model"]["time_constant_down_s"]) * perturbation.time_constant_down_scale,
        "max_rot_velocity_radps": float(reference["motor_model"]["max_rot_velocity_radps"]) * perturbation.max_rot_velocity_scale,
        "motor_constant": float(reference["motor_model"]["motor_constant"]) * perturbation.motor_constant_scale,
        "moment_constant": float(reference["motor_model"]["moment_constant"]) * perturbation.moment_constant_scale,
        "rotor_drag_coefficient": float(reference["motor_model"]["rotor_drag_coefficient"]) * perturbation.rotor_drag_scale,
        "rolling_moment_coefficient": float(reference["motor_model"]["rolling_moment_coefficient"]) * perturbation.rolling_moment_scale,
        "rotor_velocity_slowdown_sim": float(reference["motor_model"]["rotor_velocity_slowdown_sim"]),
    }


def create_offnominal_model_assets(
    *,
    px4_root: Path,
    asset_root: Path,
    model_name: str,
    perturbation: OffnominalPerturbation,
) -> dict[str, str]:
    models_root = px4_root / "Tools" / "simulation" / "gz" / "models"
    stock_model = models_root / "x500"
    stock_base = models_root / "x500_base"
    target_model = models_root / model_name
    target_base = models_root / f"{model_name}_base"

    if target_model.exists():
        shutil.rmtree(target_model)
    if target_base.exists():
        shutil.rmtree(target_base)

    shutil.copytree(stock_model, target_model)
    shutil.copytree(stock_base, target_base)

    reference = parse_x500_sdf_reference(stock_model / "model.sdf", stock_base / "model.sdf")
    offnominal = offnominal_reference(reference, perturbation)

    base_tree = ET.parse(target_base / "model.sdf")
    base_root = base_tree.getroot()
    base_model = base_root.find("model")
    if base_model is not None:
        base_model.set("name", f"{model_name}_base")
    _set_text(base_root, ".//link[@name='base_link']/inertial/mass", offnominal["mass_kg"])
    _set_text(base_root, ".//link[@name='base_link']/inertial/inertia/ixx", offnominal["ixx_kgm2"])
    _set_text(base_root, ".//link[@name='base_link']/inertial/inertia/iyy", offnominal["iyy_kgm2"])
    _set_text(base_root, ".//link[@name='base_link']/inertial/inertia/izz", offnominal["izz_kgm2"])
    _indent(base_root)
    base_tree.write(target_base / "model.sdf", encoding="utf-8", xml_declaration=True)

    base_config = ET.parse(target_base / "model.config")
    base_config_root = base_config.getroot()
    name_node = base_config_root.find("name")
    if name_node is not None:
        name_node.text = f"{model_name}_base"
    _indent(base_config_root)
    base_config.write(target_base / "model.config", encoding="utf-8", xml_declaration=True)

    model_tree = ET.parse(target_model / "model.sdf")
    model_root = model_tree.getroot()
    model_node = model_root.find("model")
    if model_node is not None:
        model_node.set("name", model_name)
    include_uri = model_root.find("./model/include/uri")
    if include_uri is not None:
        include_uri.text = f"model://{model_name}_base"
    for plugin in model_root.findall(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']"):
        _set_text(plugin, "timeConstantUp", offnominal["time_constant_up_s"])
        _set_text(plugin, "timeConstantDown", offnominal["time_constant_down_s"])
        _set_text(plugin, "maxRotVelocity", offnominal["max_rot_velocity_radps"])
        _set_text(plugin, "motorConstant", offnominal["motor_constant"])
        _set_text(plugin, "momentConstant", offnominal["moment_constant"])
        _set_text(plugin, "rotorDragCoefficient", offnominal["rotor_drag_coefficient"])
        _set_text(plugin, "rollingMomentCoefficient", offnominal["rolling_moment_coefficient"])
        _set_text(plugin, "rotorVelocitySlowdownSim", offnominal["rotor_velocity_slowdown_sim"])
    _indent(model_root)
    model_tree.write(target_model / "model.sdf", encoding="utf-8", xml_declaration=True)
    _inject_truth_logger_plugin(target_model / "model.sdf")

    model_config = ET.parse(target_model / "model.config")
    model_config_root = model_config.getroot()
    name_node = model_config_root.find("name")
    if name_node is not None:
        name_node.text = model_name
    _indent(model_config_root)
    model_config.write(target_model / "model.config", encoding="utf-8", xml_declaration=True)

    return {
        "reference_model_sdf": str((target_model / "model.sdf").resolve()),
        "reference_base_sdf": str((target_base / "model.sdf").resolve()),
        "model_name": model_name,
        "offnominal_truth": offnominal,
        "perturbation": asdict(perturbation),
        "installed_model_dir": str(target_model.resolve()),
        "installed_base_model_dir": str(target_base.resolve()),
    }


def create_light_breeze_world(*, px4_root: Path, asset_root: Path, world_name: str) -> Path:
    source = px4_root / "Tools" / "simulation" / "gz" / "worlds" / "default.sdf"
    target = px4_root / "Tools" / "simulation" / "gz" / "worlds" / f"{world_name}.sdf"
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is not None:
        world.set("name", world_name)
        wind = world.find("wind")
        if wind is None:
            wind = ET.SubElement(world, "wind")
        wind_node = wind.find("linear_velocity")
        if wind_node is None:
            wind_node = ET.SubElement(wind, "linear_velocity")
        wind_node.text = "0.6 0.2 0.0"
    _indent(root)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target


def _load_existing_manifest(path: Path) -> dict | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _prepare_model_override_from_dir(source_model_dir: Path, target_model_name: str, override_models_root: Path) -> None:
    target_dir = override_models_root / target_model_name
    if target_dir.exists():
        shutil.rmtree(target_dir)
    override_models_root.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_model_dir, target_dir)
    _inject_truth_logger_plugin(target_dir / "model.sdf")


def _build_env_with_world(
    *,
    px4_root: Path,
    build_dir: Path,
    run_rootfs: Path,
    override_models_root: Path,
    model_name: str,
    world_name: str,
    override_worlds_root: Path | None,
    headless: bool,
) -> dict[str, str]:
    env = _build_px4_env(px4_root, build_dir, run_rootfs, override_models_root, model_name, headless)
    env["PX4_GZ_WORLD"] = world_name
    if override_worlds_root is not None:
        env["PX4_GZ_WORLDS"] = str(override_worlds_root.resolve())
        world_list = [p for p in str(env.get("GZ_SIM_RESOURCE_PATH", "")).split(":") if p]
        if str(override_worlds_root.resolve()) not in world_list:
            env["GZ_SIM_RESOURCE_PATH"] = f"{override_worlds_root.resolve()}:{env['GZ_SIM_RESOURCE_PATH']}"
    return env


def _takeoff_hover_target(ground_state: dict[str, float], altitude_m: float = 5.0) -> tuple[float, float, float]:
    return (
        float(ground_state["x"]),
        float(ground_state["y"]),
        float(ground_state["z"]) - float(altitude_m),
    )


def _apply_x500_esc_scaling(session: Px4SitlSession, *, min_value: int = SITL_ESC_MIN, max_value: int = SITL_ESC_MAX) -> None:
    for motor_idx in range(1, 5):
        session.send(f"param set SIM_GZ_EC_MIN{motor_idx} {min_value}")
        session.send(f"param set SIM_GZ_EC_MAX{motor_idx} {max_value}")


def run_validation_with_assets(
    *,
    px4_root: Path,
    out_root: Path,
    label: str,
    display_name: str,
    source_model_dir: Path,
    model_name: str,
    entries: Iterable = VALIDATION_CASES,
    world_name: str = "default",
    override_worlds_root: Path | None = None,
    headless: bool = True,
) -> dict:
    build_dir = px4_root / "build" / "px4_sitl_default"
    base_rootfs = build_dir / "rootfs"
    runtime_root = out_root / "runtime" / label
    result_root = out_root / label
    results: list[dict] = []
    export_manifest = None
    for entry in entries:
        case_runtime_root = runtime_root / entry.name
        run_rootfs = case_runtime_root / "rootfs"
        override_models_root = case_runtime_root / "override_models"

        _prepare_run_rootfs(base_rootfs, run_rootfs)
        export_manifest = export_validation_trajectories(run_rootfs / "trajectories")
        _prepare_model_override_from_dir(source_model_dir, model_name, override_models_root)
        env = _build_env_with_world(
            px4_root=px4_root,
            build_dir=build_dir,
            run_rootfs=run_rootfs,
            override_models_root=override_models_root,
            model_name=model_name,
            world_name=world_name,
            override_worlds_root=override_worlds_root,
            headless=headless,
        )

        _cleanup_stale_sitl_processes()
        session = Px4SitlSession(px4_root, run_rootfs, env)
        try:
            session.start()
            _apply_x500_esc_scaling(session)
            session.send("param set MIS_TAKEOFF_ALT 5.0")
            session.send("custom_pos_control start")
            session.send("trajectory_reader start")
            session.send("custom_pos_control set px4_default")
            session.send("custom_pos_control enable")
            session.send("trajectory_reader set_mode position")
            session.expect("Ready for takeoff!", timeout_s=30)
            ground_state = session.sample_local_position()
            ground_anchor = (
                float(ground_state["x"]),
                float(ground_state["y"]),
                float(ground_state["z"]),
            )
            session.send(f"trajectory_reader abs_ref {ground_anchor[0]} {ground_anchor[1]} {ground_anchor[2]} 0")
            session.arm()
            time.sleep(1.0)
            session.send_no_wait("commander takeoff")
            hover_target = _takeoff_hover_target(ground_state, altitude_m=5.0)
            session.wait_until_position(hover_target, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=45.0)
            session.sync_prompt(timeout_s=10.0)
            common_anchor = hover_target
            session.send(f"trajectory_reader abs_ref {common_anchor[0]} {common_anchor[1]} {common_anchor[2]} 0")
            session.send_no_wait("commander mode offboard")
            session.wait_until_position(common_anchor, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=20.0)
            time.sleep(2.0)
            session.wait_until_position(common_anchor, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=10.0)
            session.sync_prompt(timeout_s=10.0)
            session.send(f"trajectory_reader set_traj_anchor {common_anchor[0]} {common_anchor[1]} {common_anchor[2]}")
            session.send(f"trajectory_reader set_traj_id {entry.traj_id}")
            session.send("trajectory_reader set_mode trajectory")
            match = session.expect(TRACKING_LOG_START_RE, timeout_s=20)
            session.expect(
                "Trajectory EOF reached, tracking log closed",
                timeout_s=max(40.0, _trajectory_duration_s(entry) + 15.0),
            )
            assert match is not None
            copied = _copy_log(
                _find_tracking_log(run_rootfs, match.group(1)),
                result_root / "tracking_logs" / f"{entry.name}.csv",
            )
            results.append(
                {
                    "traj_id": entry.traj_id,
                    "name": entry.name,
                    "duration_s": _trajectory_duration_s(entry),
                    "tracking_log": copied,
                    "console_log": str((run_rootfs / "px4_console.log").resolve()),
                }
            )
            session.send_no_wait("commander mode auto:land")
            session.wait_until_landed(ground_z_m=ground_anchor[2], timeout_s=45.0)
            session.sync_prompt(timeout_s=10.0)
        finally:
            session.shutdown()
            _cleanup_stale_sitl_processes()

    payload = {
        "label": label,
        "display_name": display_name,
        "model_name": model_name,
        "world_name": world_name,
        "export_manifest": export_manifest,
        "results": results,
    }
    result_root.mkdir(parents=True, exist_ok=True)
    (result_root / "run_manifest.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def run_identification_with_assets(
    *,
    px4_root: Path,
    out_root: Path,
    label: str,
    source_model_dir: Path,
    model_name: str,
    headless: bool = True,
) -> dict:
    build_dir = px4_root / "build" / "px4_sitl_default"
    base_rootfs = build_dir / "rootfs"
    runtime_root = out_root / "runtime" / label
    result_root = out_root / label
    results: list[dict] = []
    truth_csvs: list[str] = []
    for idx, profile in enumerate(IDENT_PROFILES, start=1):
        profile_runtime_root = runtime_root / f"{idx:02d}_{profile}"
        run_rootfs = profile_runtime_root / "rootfs"
        override_models_root = profile_runtime_root / "override_models"
        _prepare_run_rootfs(base_rootfs, run_rootfs)
        _prepare_model_override_from_dir(source_model_dir, model_name, override_models_root)
        env = _build_env_with_world(
            px4_root=px4_root,
            build_dir=build_dir,
            run_rootfs=run_rootfs,
            override_models_root=override_models_root,
            model_name=model_name,
            world_name="default",
            override_worlds_root=None,
            headless=headless,
        )

        _cleanup_stale_sitl_processes()
        session = Px4SitlSession(px4_root, run_rootfs, env)
        try:
            session.start()
            _apply_x500_esc_scaling(session)
            session.send("param set MIS_TAKEOFF_ALT 5.0")
            session.send("custom_pos_control start")
            session.send("trajectory_reader start")
            session.send("custom_pos_control enable")
            session.send("custom_pos_control set sysid")
            session.send("trajectory_reader set_mode position")
            session.expect("Ready for takeoff!", timeout_s=30)
            ground = session.sample_local_position()
            ground_anchor = (
                float(ground["x"]),
                float(ground["y"]),
                float(ground["z"]),
            )
            session.send(f"trajectory_reader abs_ref {ground_anchor[0]} {ground_anchor[1]} {ground_anchor[2]} 0")
            session.arm()
            time.sleep(1.0)
            session.send_no_wait("commander takeoff")
            hover_target = _takeoff_hover_target(ground, altitude_m=5.0)
            session.wait_until_position(hover_target, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=45.0)
            session.sync_prompt(timeout_s=10.0)
            common_anchor = hover_target
            session.send(f"trajectory_reader abs_ref {common_anchor[0]} {common_anchor[1]} {common_anchor[2]} 0")
            session.send_no_wait("commander mode offboard")
            session.wait_until_position(common_anchor, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=20.0)
            time.sleep(2.0)
            session.wait_until_position(common_anchor, xy_tol_m=0.25, z_tol_m=0.25, timeout_s=10.0)
            session.sync_prompt(timeout_s=10.0)
            session.send(f"trajectory_reader abs_ref {common_anchor[0]} {common_anchor[1]} {common_anchor[2]} 0")
            session.send(f"trajectory_reader set_ident_profile {profile}")
            session.send("trajectory_reader set_mode identification")
            match_track = session.expect(TRACKING_LOG_START_RE, timeout_s=20)
            match_ident = session.expect(IDENT_LOG_START_RE, timeout_s=5)
            session.expect(f"Identification maneuver completed: {profile}", timeout_s=70.0)
            session.expect(f"Identification log completed: {profile}", timeout_s=20.0)
            session.expect(f"Tracking log completed: {profile}", timeout_s=20.0)
            assert match_ident is not None and match_track is not None
            sortie_root = result_root / f"{idx:02d}_{profile}"
            eval_path = sortie_root / "identification_traces" / "eval_00000.csv"
            track_path = sortie_root / "tracking_logs" / "run_00000.csv"
            _copy_log(_find_tracking_log(run_rootfs, match_ident.group(1)), eval_path)
            _copy_log(_find_tracking_log(run_rootfs, match_track.group(1)), track_path)
            truth_logs = sorted((run_rootfs / "sysid_truth_logs").glob("*.csv"))
            if not truth_logs:
                raise RuntimeError(f"No sysid truth CSV produced for {profile}")
            truth_csv = truth_logs[-1]
            dst = sortie_root / "gazebo_truth_traces" / "eval_00000.csv"
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(truth_csv, dst)
            truth_csvs.append(str(dst.resolve()))
            results.append(
                {
                    "profile": profile,
                    "identification_log": str(eval_path.resolve()),
                    "tracking_log": str(track_path.resolve()),
                    "truth_log": str(dst.resolve()),
                    "console_log": str((run_rootfs / "px4_console.log").resolve()),
                }
            )
            session.send_no_wait("commander mode auto:land")
            session.wait_until_landed(ground_z_m=ground_anchor[2], timeout_s=45.0)
            session.sync_prompt(timeout_s=10.0)
        finally:
            session.shutdown()
            _cleanup_stale_sitl_processes()

    payload = {
        "label": label,
        "model_name": model_name,
        "truth_csvs": truth_csvs,
        "results": results,
    }
    result_root.mkdir(parents=True, exist_ok=True)
    (result_root / "run_manifest.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def _load_tracking(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(csv_path)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(dtype=float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(dtype=float)
    ref[:, 2] *= -1.0
    pos[:, 2] *= -1.0
    return ref, pos


def _trajectory_rmse(ref: np.ndarray, pos: np.ndarray) -> tuple[float, np.ndarray]:
    delta = pos - ref
    sq = np.sum(delta ** 2, axis=1)
    return float(np.sqrt(np.mean(sq))), np.sqrt(sq)


def _runaway_cutoff_index(
    ref: np.ndarray,
    pos: np.ndarray,
    *,
    z_error_threshold_m: float = 6.0,
    consecutive_samples: int = 12,
) -> int:
    if len(ref) == 0 or len(pos) == 0:
        return 0

    usable = min(len(ref), len(pos))
    z_error = np.abs(pos[:usable, 2] - ref[:usable, 2])
    run = 0

    for idx, value in enumerate(z_error):
        if value > z_error_threshold_m:
            run += 1
            if run >= consecutive_samples:
                return max(1, idx - consecutive_samples + 1)
        else:
            run = 0

    return usable


def _trim_triplet(ref: np.ndarray, stock: np.ndarray, real: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    usable = min(len(ref), len(stock), len(real))
    cutoff = min(
        _runaway_cutoff_index(ref[:usable], stock[:usable]),
        _runaway_cutoff_index(ref[:usable], real[:usable]),
    )
    cutoff = max(1, cutoff)
    return ref[:cutoff], stock[:cutoff], real[:cutoff]


def _set_axes_equal(ax: plt.Axes, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = max(float(np.max(maxs - mins)) / 2.0, 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def build_offnominal_figures(
    *,
    stock_root: Path,
    offnominal_root: Path,
    out_dir: Path,
) -> dict[str, str]:
    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.family": "DejaVu Serif",
            "font.size": 15,
            "axes.titlesize": 18,
            "axes.labelsize": 16,
            "legend.fontsize": 15,
            "xtick.labelsize": 13,
            "ytick.labelsize": 13,
        }
    )
    plt.style.use("seaborn-v0_8-whitegrid")

    out_dir.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, str] = {}

    for figure_index, cases in enumerate(PANEL_GROUPS, start=1):
        cols = len(cases)
        width_ratios = [1.0] * cols + [0.26, 0.07, 0.07]
        fig = plt.figure(figsize=(6.6 * cols + 3.0, 7.4), layout="constrained")
        gs = fig.add_gridspec(1, cols + 3, width_ratios=width_ratios)
        axes = [fig.add_subplot(gs[0, idx], projection="3d") for idx in range(cols)]
        legend_ax = fig.add_subplot(gs[0, cols])
        cax_stock = fig.add_subplot(gs[0, cols + 1])
        cax_real = fig.add_subplot(gs[0, cols + 2])
        legend_ax.axis("off")

        global_max_stock_error = 1e-6
        global_max_real_error = 1e-6
        loaded = {}
        for case in cases:
            stock_ref, stock_pos = _load_tracking(stock_root / "tracking_logs" / f"{case}.csv")
            real_ref, real_pos = _load_tracking(offnominal_root / "tracking_logs" / f"{case}.csv")
            ref, stock_pos, real_pos = _trim_triplet(stock_ref, stock_pos, real_pos)
            rmse_real, err_real = _trajectory_rmse(ref, real_pos)
            rmse_stock, err_stock = _trajectory_rmse(ref, stock_pos)
            global_max_stock_error = max(global_max_stock_error, float(np.max(err_stock)))
            global_max_real_error = max(global_max_real_error, float(np.max(err_real)))
            loaded[case] = {
                "stock_ref": ref,
                "stock_pos": stock_pos,
                "real_pos": real_pos,
                "rmse_stock": rmse_stock,
                "rmse_real": rmse_real,
                "err_stock": err_stock,
                "err_real": err_real,
                "samples": len(ref),
            }

        norm_stock = mpl.colors.Normalize(vmin=0.0, vmax=global_max_stock_error)
        norm_real = mpl.colors.Normalize(vmin=0.0, vmax=global_max_real_error)
        cmap_stock = plt.get_cmap("viridis")
        cmap_real = plt.get_cmap("turbo")

        for ax, case in zip(axes, cases):
            payload = loaded[case]
            ref = payload["stock_ref"]
            stock = payload["stock_pos"]
            real = payload["real_pos"]
            err_stock = payload["err_stock"]
            err_real = payload["err_real"]

            ax.plot(
                ref[:, 0],
                ref[:, 1],
                ref[:, 2],
                linestyle="--",
                linewidth=2.2,
                color="#304c89",
                label="Reference",
            )
            if len(stock) >= 2:
                stock_segments = np.stack([stock[:-1], stock[1:]], axis=1)
                stock_segment_error = 0.5 * (err_stock[:-1] + err_stock[1:])
                stock_lc = Line3DCollection(stock_segments, cmap=cmap_stock, norm=norm_stock, linewidth=2.8)
                stock_lc.set_array(stock_segment_error)
                ax.add_collection3d(stock_lc)
            else:
                ax.scatter(stock[:, 0], stock[:, 1], stock[:, 2], c=[cmap_stock(norm_stock(float(err_stock[0])))], s=25)
            if len(real) >= 2:
                segments = np.stack([real[:-1], real[1:]], axis=1)
                segment_error = 0.5 * (err_real[:-1] + err_real[1:])
                lc = Line3DCollection(segments, cmap=cmap_real, norm=norm_real, linewidth=3.0)
                lc.set_array(segment_error)
                ax.add_collection3d(lc)
            else:
                ax.scatter(real[:, 0], real[:, 1], real[:, 2], c=[cmap_real(norm_real(float(err_real[0])))], s=25)

            all_points = np.vstack([ref, stock, real])
            _set_axes_equal(ax, all_points)
            ax.set_title(
                f"{case}\nRMSE SITL={payload['rmse_stock']:.3f} m | Real proxy={payload['rmse_real']:.3f} m"
            )
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Z (up) [m]")
            ax.view_init(elev=22, azim=-60)

        legend_handles = [
            mpl.lines.Line2D([0], [0], color="#304c89", linestyle="--", linewidth=2.2, label="Reference"),
            mpl.lines.Line2D([0], [0], color=cmap_stock(0.72), linestyle="-", linewidth=2.8, label="SITL"),
            mpl.lines.Line2D([0], [0], color="#d1495b", linestyle="-", linewidth=3.0, label="Real flight results"),
        ]
        legend_ax.legend(handles=legend_handles, loc="upper left", frameon=True, borderpad=1.0, labelspacing=1.0)
        scalar_stock = mpl.cm.ScalarMappable(norm=norm_stock, cmap=cmap_stock)
        scalar_real = mpl.cm.ScalarMappable(norm=norm_real, cmap=cmap_real)
        cbar_stock = fig.colorbar(scalar_stock, cax=cax_stock)
        cbar_real = fig.colorbar(scalar_real, cax=cax_real)
        cbar_stock.set_label("SITL instantaneous position error [m]")
        cbar_real.set_label("Real flight results instantaneous position error [m]")
        cbar_stock.ax.tick_params(labelsize=13)
        cbar_real.ax.tick_params(labelsize=13)

        name = "group_1_circle_hairpin_lemniscate" if figure_index == 1 else "group_2_time_optimal_minimum_snap"
        out_path = out_dir / f"{name}.png"
        fig.savefig(out_path, bbox_inches="tight")
        plt.close(fig)
        outputs[name] = str(out_path.resolve())

    return outputs


def run_offnominal_study(
    *,
    px4_root: Path,
    out_dir: Path,
    headless: bool = True,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    asset_root = out_dir / "runtime_assets"
    world_name = "offnominal_breeze"
    model_name = "x500_offnominal"
    perturbation = OffnominalPerturbation()

    stock_model_dir = px4_root / "Tools" / "simulation" / "gz" / "models" / "x500"
    offnominal_meta = create_offnominal_model_assets(
        px4_root=px4_root,
        asset_root=asset_root,
        model_name=model_name,
        perturbation=perturbation,
    )
    breeze_world = create_light_breeze_world(px4_root=px4_root, asset_root=asset_root, world_name=world_name)

    results_root = out_dir / "results"
    stock_manifest_path = results_root / "stock_baseline_pid" / "run_manifest.json"
    ident_manifest_path = results_root / "offnominal_identification" / "run_manifest.json"
    offnominal_manifest_path = results_root / "offnominal_windy_pid" / "run_manifest.json"

    stock_manifest = _load_existing_manifest(stock_manifest_path)
    if stock_manifest is None:
        stock_manifest = run_validation_with_assets(
            px4_root=px4_root,
            out_root=results_root,
            label="stock_baseline_pid",
            display_name="Stock x500 baseline PID",
            source_model_dir=stock_model_dir,
            model_name="x500",
            world_name="default",
            headless=headless,
        )

    ident_manifest = _load_existing_manifest(ident_manifest_path)
    if ident_manifest is None:
        ident_manifest = run_identification_with_assets(
            px4_root=px4_root,
            out_root=results_root,
            label="offnominal_identification",
            source_model_dir=px4_root / "Tools" / "simulation" / "gz" / "models" / model_name,
            model_name=model_name,
            headless=headless,
        )

    candidate_dir = out_dir / "candidate_offnominal_recovered"
    compare_cmd = [
        sys.executable,
        str(REPO_ROOT / "experimental_validation" / "compare_with_sdf.py"),
        "--results-root",
        str((out_dir / "results" / "offnominal_identification").resolve()),
        "--out-dir",
        str(candidate_dir.resolve()),
        "--sdf-model",
        offnominal_meta["reference_model_sdf"],
        "--sdf-base-model",
        offnominal_meta["reference_base_sdf"],
    ]
    import subprocess

    proc = subprocess.run(compare_cmd, cwd=str(REPO_ROOT), text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f"compare_with_sdf failed\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}")
    compare_summary = json.loads(proc.stdout)

    offnominal_manifest = _load_existing_manifest(offnominal_manifest_path)
    if offnominal_manifest is None:
        offnominal_manifest = run_validation_with_assets(
            px4_root=px4_root,
            out_root=results_root,
            label="offnominal_windy_pid",
            display_name="Off-nominal x500 with light wind",
            source_model_dir=px4_root / "Tools" / "simulation" / "gz" / "models" / model_name,
            model_name=model_name,
            world_name=world_name,
            override_worlds_root=None,
            headless=headless,
        )

    figures = build_offnominal_figures(
        stock_root=out_dir / "results" / "stock_baseline_pid",
        offnominal_root=out_dir / "results" / "offnominal_windy_pid",
        out_dir=out_dir / "figures",
    )

    summary = {
        "stock_manifest": stock_manifest,
        "identification_manifest": ident_manifest,
        "offnominal_manifest": offnominal_manifest,
        "offnominal_reference": offnominal_meta,
        "wind_world": str(breeze_world.resolve()),
        "candidate_dir": str(candidate_dir.resolve()),
        "compare_summary": compare_summary,
        "figures": figures,
    }
    (out_dir / "offnominal_study_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    parser.add_argument("--out-dir", default="~/px4-system-identification/examples/offnominal_sitl_study")
    parser.add_argument("--gui", action="store_true", help="Run Gazebo with GUI.")
    args = parser.parse_args()

    payload = run_offnominal_study(
        px4_root=Path(args.px4_root).expanduser().resolve(),
        out_dir=Path(args.out_dir).expanduser().resolve(),
        headless=not args.gui,
    )
    print(json.dumps({"ok": True, "summary": str((Path(args.out_dir).expanduser().resolve() / 'offnominal_study_summary.json'))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
