#!/usr/bin/env python3
"""Generate paper-ready digital-twin validation artifacts.

The current version can work in two modes:
1. Synthetic/noisy validation data for paper drafting before real-flight logs exist.
2. Re-generation with a later candidate identified_parameters.json.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from experimental_validation.reference_models import default_candidate_identified, default_x500_reference
from experimental_validation.twin_metrics import (
    build_blended_twin_score_from_values,
    flatten_identified_metrics,
    flatten_reference_metrics,
)


@dataclass(frozen=True)
class TrajectoryCase:
    name: str
    duration_s: float
    scenario: dict[str, float]


DEFAULT_TRAJECTORIES = (
    TrajectoryCase("hairpin", 28.0, {"payload_mass": 0.12, "com_z": 0.01, "tau_scale": 1.02}),
    TrajectoryCase("lemniscate", 30.0, {"payload_mass": 0.10, "com_z": 0.01, "tau_scale": 1.02}),
    TrajectoryCase("circle", 30.0, {"payload_mass": 0.14, "com_x": 0.01, "com_y": -0.01}),
    TrajectoryCase("time_optimal_30s", 30.0, {"payload_mass": 0.24, "com_x": 0.02, "com_y": -0.01, "com_z": 0.03, "tau_scale": 1.04}),
    TrajectoryCase("minimum_snap_50s", 50.0, {"payload_mass": 0.18, "com_x": 0.01, "com_y": 0.01, "com_z": 0.02, "arm_scale": 1.01}),
)


def _interp_waypoints(
    t: np.ndarray,
    duration_s: float,
    waypoints: list[tuple[float, float, float]],
    *,
    knot_times: list[float] | None = None,
    smooth_window: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if knot_times is None:
        knot_times = np.linspace(0.0, 1.0, len(waypoints)).tolist()
    s = np.clip(t / max(duration_s, 1e-6), 0.0, 1.0)
    xs = np.interp(s, knot_times, [p[0] for p in waypoints])
    ys = np.interp(s, knot_times, [p[1] for p in waypoints])
    zs = np.interp(s, knot_times, [p[2] for p in waypoints])
    if smooth_window > 1:
        kernel = np.ones(smooth_window, dtype=float) / float(smooth_window)
        xs = np.convolve(xs, kernel, mode="same")
        ys = np.convolve(ys, kernel, mode="same")
        zs = np.convolve(zs, kernel, mode="same")
    return xs, ys, zs


def _load_candidate(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return default_candidate_identified()
    path = Path(path)
    file_path = path / "identified_parameters.json" if path.is_dir() else path
    return json.loads(file_path.read_text(encoding="utf-8"))


def _effective_metric_vector(
    base: dict[str, float],
    *,
    payload_mass: float = 0.0,
    com_x: float = 0.0,
    com_y: float = 0.0,
    com_z: float = 0.0,
    arm_scale: float = 1.0,
    motor_constant_scale: float = 1.0,
    tau_scale: float = 1.0,
) -> dict[str, float]:
    payload_mass = float(payload_mass)
    com_x = float(com_x)
    com_y = float(com_y)
    com_z = float(com_z)
    arm_scale = float(arm_scale)
    motor_constant_scale = float(motor_constant_scale)
    tau_scale = float(tau_scale)

    mass = base["mass_kg"] + payload_mass
    ixx = base["ixx_kgm2"] * arm_scale ** 2 + payload_mass * (com_y ** 2 + com_z ** 2)
    iyy = base["iyy_kgm2"] * arm_scale ** 2 + payload_mass * (com_x ** 2 + com_z ** 2)
    izz = base["izz_kgm2"] * arm_scale ** 2 + payload_mass * (com_x ** 2 + com_y ** 2)

    return {
        "mass_kg": mass,
        "ixx_kgm2": ixx,
        "iyy_kgm2": iyy,
        "izz_kgm2": izz,
        "time_constant_up_s": base["time_constant_up_s"] * tau_scale,
        "time_constant_down_s": base["time_constant_down_s"] * tau_scale,
        "max_rot_velocity_radps": base["max_rot_velocity_radps"] / math.sqrt(max(motor_constant_scale, 1e-6)),
        "motor_constant": base["motor_constant"] * motor_constant_scale,
        "moment_constant": base["moment_constant"] * motor_constant_scale,
        "rotor_drag_coefficient": base["rotor_drag_coefficient"] * (1.0 + 0.15 * (arm_scale - 1.0)),
        "rolling_moment_coefficient": base["rolling_moment_coefficient"] * (1.0 + 0.20 * (arm_scale - 1.0)),
        "rotor_velocity_slowdown_sim": base["rotor_velocity_slowdown_sim"],
    }


def _scenario_difficulty(**scenario: float) -> float:
    payload_mass = float(scenario.get("payload_mass", 0.0))
    com_x = abs(float(scenario.get("com_x", 0.0)))
    com_y = abs(float(scenario.get("com_y", 0.0)))
    com_z = abs(float(scenario.get("com_z", 0.0)))
    arm_scale = abs(float(scenario.get("arm_scale", 1.0)) - 1.0)
    motor_constant_scale = abs(float(scenario.get("motor_constant_scale", 1.0)) - 1.0)
    tau_scale = abs(float(scenario.get("tau_scale", 1.0)) - 1.0)
    return (
        0.18 * payload_mass
        + 1.25 * com_z
        + 0.80 * (com_x + com_y)
        + 3.00 * arm_scale
        + 1.10 * motor_constant_scale
        + 0.85 * tau_scale
    )


def _mission_scores(reference_metrics: dict[str, float], candidate_metrics: dict[str, float], **scenario: float) -> tuple[float, float, dict[str, Any]]:
    ref_effective = _effective_metric_vector(reference_metrics, **scenario)
    # In the stress tests, the identified twin remains fixed while the
    # reference plant is perturbed. This makes the plots reflect model
    # robustness instead of trivially applying the same perturbation to both.
    cand_effective = dict(candidate_metrics)
    difficulty = _scenario_difficulty(**scenario)
    match = build_blended_twin_score_from_values(cand_effective, ref_effective)
    real_score = 100.0 * math.exp(-difficulty)
    twin_score = real_score * (match["score"] / 100.0)
    payload = {
        "real_score": real_score,
        "twin_score": twin_score,
        "difficulty": difficulty,
        "match": match,
        "scenario": {key: float(value) for key, value in scenario.items()},
    }
    return real_score, twin_score, payload


def _trajectory_reference(case: TrajectoryCase, samples: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t = np.linspace(0.0, case.duration_s, samples)
    w = 2.0 * np.pi / case.duration_s
    if case.name == "hairpin":
        x, y, z = _interp_waypoints(
            t,
            case.duration_s,
            [
                (-4.2, -1.6, -3.00),
                (4.0, -1.6, -3.00),
                (4.6, -0.2, -3.10),
                (4.0, 1.6, -3.18),
                (-4.2, 1.6, -3.18),
            ],
            smooth_window=19,
        )
    elif case.name == "lemniscate":
        x = 2.8 * np.sin(w * t)
        y = 1.6 * np.sin(w * t) * np.cos(w * t)
        z = -3.0 - 0.35 * np.sin(0.5 * w * t)
    elif case.name == "circle":
        radius = 2.6
        x = radius * np.cos(w * t)
        y = radius * np.sin(w * t)
        z = -3.05 - 0.18 * np.sin(0.5 * w * t)
    elif case.name == "time_optimal_30s":
        x, y, z = _interp_waypoints(
            t,
            case.duration_s,
            [
                (-3.5, -1.8, -2.9),
                (-1.0, 1.6, -3.0),
                (2.6, 1.9, -3.25),
                (4.2, -0.4, -3.35),
                (1.8, -2.0, -3.10),
                (-1.8, -0.8, -2.85),
                (-4.0, 1.4, -3.05),
                (-3.5, -1.8, -2.9),
            ],
            knot_times=[0.0, 0.08, 0.18, 0.30, 0.45, 0.62, 0.82, 1.0],
            smooth_window=11,
        )
    elif case.name == "minimum_snap_50s":
        x, y, z = _interp_waypoints(
            t,
            case.duration_s,
            [
                (-3.8, -1.2, -2.9),
                (-2.2, 0.8, -3.0),
                (0.0, 2.2, -3.2),
                (2.4, 1.6, -3.35),
                (3.5, -0.2, -3.25),
                (2.1, -1.8, -3.1),
                (-0.5, -2.3, -3.0),
                (-3.0, -0.8, -2.95),
                (-3.8, -1.2, -2.9),
            ],
            knot_times=[0.0, 0.10, 0.22, 0.36, 0.50, 0.66, 0.80, 0.92, 1.0],
            smooth_window=29,
        )
    else:
        raise ValueError(f"unsupported trajectory case: {case.name}")
    return t, x, y, z


def _low_frequency_noise(rng: np.random.Generator, samples: int, scale: float) -> np.ndarray:
    raw = rng.normal(0.0, scale, size=samples)
    kernel = np.ones(21) / 21.0
    return np.convolve(raw, kernel, mode="same")


def _simulate_track(
    t: np.ndarray,
    x_ref: np.ndarray,
    y_ref: np.ndarray,
    z_ref: np.ndarray,
    *,
    score: float,
    match_penalty: float,
    rng: np.random.Generator,
    variant: str,
) -> dict[str, np.ndarray]:
    dt = float(t[1] - t[0])
    lag_s = 0.05 if variant == "real" else 0.08 + 0.45 * match_penalty
    lag_steps = int(round(lag_s / max(dt, 1e-6)))
    amp_xy = 1.0 - (0.006 if variant == "real" else 0.015 + 0.045 * match_penalty)
    amp_z = 1.0 - (0.004 if variant == "real" else 0.010 + 0.030 * match_penalty)
    bias_xy = 0.015 if variant == "real" else 0.035 * match_penalty
    bias_z = -0.01 if variant == "real" else 0.05 * match_penalty

    x = np.roll(x_ref, lag_steps) * amp_xy + bias_xy + _low_frequency_noise(rng, len(t), 0.015 if variant == "real" else 0.025)
    y = np.roll(y_ref, lag_steps) * amp_xy - bias_xy + _low_frequency_noise(rng, len(t), 0.015 if variant == "real" else 0.025)
    z = np.roll(z_ref, lag_steps) * amp_z + bias_z + _low_frequency_noise(rng, len(t), 0.010 if variant == "real" else 0.020)

    error = np.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2 + (z - z_ref) ** 2)
    return {
        "t": t,
        "x": x,
        "y": y,
        "z": z,
        "error_norm": error,
        "score_hint": np.full_like(t, score),
    }


def _write_track_csv(path: Path, track: dict[str, np.ndarray]) -> None:
    header = "t_s,x_m,y_m,z_m,error_norm_m,score_hint\n"
    rows = [header]
    for i in range(len(track["t"])):
        rows.append(
            f"{track['t'][i]:.6f},{track['x'][i]:.6f},{track['y'][i]:.6f},{track['z'][i]:.6f},{track['error_norm'][i]:.6f},{track['score_hint'][i]:.6f}\n"
        )
    path.write_text("".join(rows), encoding="utf-8")


def _plot_overlay(out_path: Path, case: TrajectoryCase, ref_track: dict[str, np.ndarray], real_track: dict[str, np.ndarray], twin_track: dict[str, np.ndarray], summary: dict[str, Any]) -> None:
    fig = plt.figure(figsize=(12, 5))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot(ref_track["x"], ref_track["y"], ref_track["z"], color="0.4", linestyle="--", label="Reference mission")
    ax.plot(real_track["x"], real_track["y"], real_track["z"], color="tab:blue", label="Synthetic real flight")
    ax.plot(twin_track["x"], twin_track["y"], twin_track["z"], color="tab:orange", label="Digital twin simulation")
    ax.set_title(case.name.replace("_", " ").title())
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(real_track["t"], real_track["error_norm"], color="tab:blue", label="Real vs reference")
    ax2.plot(twin_track["t"], twin_track["error_norm"], color="tab:orange", label="Twin vs reference")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Position error norm [m]")
    ax2.set_title(f"Overlay RMSE real={summary['real_rmse_m']:.3f} m | twin={summary['twin_rmse_m']:.3f} m")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _rmse(track: dict[str, np.ndarray]) -> float:
    return float(np.sqrt(np.mean(track["error_norm"] ** 2)))


def _grid_scores(
    reference_metrics: dict[str, float],
    candidate_metrics: dict[str, float],
    x_values: np.ndarray,
    y_values: np.ndarray,
    scenario_builder,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    X, Y = np.meshgrid(x_values, y_values, indexing="xy")
    match_surface = np.zeros_like(X, dtype=float)
    penalty_surface = np.zeros_like(X, dtype=float)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            scenario = scenario_builder(float(X[i, j]), float(Y[i, j]))
            real_score, twin_score, payload = _mission_scores(reference_metrics, candidate_metrics, **scenario)
            match_surface[i, j] = float(payload["match"]["score"])
            penalty_surface[i, j] = 100.0 * max(0.0, 1.0 - (twin_score / max(real_score, 1e-6)))
    return X, Y, match_surface, penalty_surface


def _write_surface_csv(path: Path, X: np.ndarray, Y: np.ndarray, match_surface: np.ndarray, penalty_surface: np.ndarray) -> None:
    lines = ["x_axis,y_axis,twin_similarity_score,expected_tracking_penalty_pct\n"]
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            lines.append(
                f"{X[i, j]:.6f},{Y[i, j]:.6f},{match_surface[i, j]:.6f},{penalty_surface[i, j]:.6f}\n"
            )
    path.write_text("".join(lines), encoding="utf-8")


def _plot_surface(out_path: Path, title: str, xlabel: str, ylabel: str, X: np.ndarray, Y: np.ndarray, match_surface: np.ndarray, penalty_surface: np.ndarray) -> None:
    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    surface = ax.plot_surface(X, Y, match_surface, cmap="viridis", linewidth=0.0, antialiased=True)
    ax.set_title(
        f"{title}\nMean twin similarity = {float(np.mean(match_surface)):.2f}% | "
        f"Mean tracking penalty = {float(np.mean(penalty_surface)):.2f}%"
    )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel("Twin similarity score [%]")
    fig.colorbar(surface, shrink=0.72, aspect=20, pad=0.08, label="Twin similarity score [%]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_line_family(
    out_path: Path,
    title: str,
    xlabel: str,
    x_values: np.ndarray,
    y_values: np.ndarray,
    match_surface: np.ndarray,
    y_label_fmt,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 5))
    if len(y_values) >= 3:
        indices = [0, len(y_values) // 2, len(y_values) - 1]
    else:
        indices = list(range(len(y_values)))
    for idx in indices:
        ax.plot(x_values, match_surface[idx, :], linewidth=2.0, label=y_label_fmt(float(y_values[idx])))
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Twin similarity score [%]")
    ax.set_ylim(0.0, 102.0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_parameter_error_bars(
    out_path: Path,
    candidate_metrics: dict[str, float],
    reference_metrics: dict[str, float],
) -> None:
    labels = [
        ("mass_kg", "Mass"),
        ("ixx_kgm2", "Ixx"),
        ("iyy_kgm2", "Iyy"),
        ("izz_kgm2", "Izz"),
        ("time_constant_up_s", "Tau up"),
        ("time_constant_down_s", "Tau down"),
        ("motor_constant", "Motor k"),
        ("moment_constant", "Moment k"),
        ("rotor_drag_coefficient", "Rotor drag"),
        ("rolling_moment_coefficient", "Roll moment"),
    ]
    names = []
    values = []
    for key, label in labels:
        ref = float(reference_metrics[key])
        ident = float(candidate_metrics[key])
        pct = 0.0 if abs(ref) < 1e-12 else abs((ident - ref) / ref) * 100.0
        names.append(label)
        values.append(pct)

    fig, ax = plt.subplots(figsize=(11, 5))
    bars = ax.bar(names, values, color="tab:green", alpha=0.82)
    ax.set_ylabel("Absolute parameter error [%]")
    ax.set_title("Comparable SDF parameter errors for the identified twin")
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_ylim(0.0, max(1.0, max(values) * 1.25))
    for bar, value in zip(bars, values, strict=False):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height(),
            f"{value:.3g}%",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    plt.setp(ax.get_xticklabels(), rotation=25, ha="right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_family_score_bars(out_path: Path, family_scores: dict[str, float]) -> None:
    names = list(family_scores.keys())
    values = [float(family_scores[name]) for name in names]
    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(names, values, color="tab:blue", alpha=0.82)
    ax.set_ylabel("Family score [%]")
    ax.set_title("Blended twin score by parameter family")
    ax.set_ylim(0.0, 102.0)
    ax.grid(True, axis="y", alpha=0.25)
    for bar, value in zip(bars, values, strict=False):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height(),
            f"{value:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_trajectory_match_scores(out_path: Path, trajectory_summaries: list[dict[str, Any]]) -> None:
    names = [item["name"] for item in trajectory_summaries]
    real_scores = [float(item["real_score"]) for item in trajectory_summaries]
    twin_scores = [float(item["twin_score"]) for item in trajectory_summaries]
    match_scores = [float(item["match_score"]) for item in trajectory_summaries]

    x = np.arange(len(names))
    width = 0.24
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x - width, real_scores, width=width, label="Synthetic real-flight score", color="tab:blue", alpha=0.85)
    ax.bar(x, twin_scores, width=width, label="Digital twin score", color="tab:orange", alpha=0.85)
    ax.bar(x + width, match_scores, width=width, label="Twin similarity score", color="tab:green", alpha=0.85)
    ax.set_xticks(x, names)
    ax.set_ylim(0.0, 102.0)
    ax.set_ylabel("Score [%]")
    ax.set_title("Validation trajectory summary scores")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="best")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def generate_paper_artifacts(
    out_dir: str | Path,
    *,
    candidate_json: str | Path | None = None,
    samples_per_traj: int = 600,
    grid_points: int = 10,
    seed: int = 17,
) -> dict[str, Any]:
    out_dir = Path(out_dir).resolve()
    figures_dir = out_dir / "figures"
    data_dir = out_dir / "data"
    figures_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    reference = default_x500_reference()
    candidate = _load_candidate(candidate_json)
    reference_metrics = flatten_reference_metrics(reference)
    candidate_metrics = flatten_identified_metrics(candidate)
    base_match = build_blended_twin_score_from_values(candidate_metrics, reference_metrics)
    _plot_parameter_error_bars(figures_dir / "parameter_error_bars.png", candidate_metrics, reference_metrics)
    _plot_family_score_bars(figures_dir / "family_score_bars.png", base_match["family_scores"])

    rng = np.random.default_rng(seed)
    trajectory_summaries: list[dict[str, Any]] = []
    for case in DEFAULT_TRAJECTORIES:
        t, x_ref, y_ref, z_ref = _trajectory_reference(case, samples_per_traj)
        _, _, payload = _mission_scores(reference_metrics, candidate_metrics, **case.scenario)
        match_penalty = 1.0 - (payload["match"]["score"] / 100.0)
        ref_track = {"t": t, "x": x_ref, "y": y_ref, "z": z_ref, "error_norm": np.zeros_like(t), "score_hint": np.full_like(t, payload["real_score"])}
        real_track = _simulate_track(t, x_ref, y_ref, z_ref, score=payload["real_score"], match_penalty=0.03, rng=rng, variant="real")
        twin_track = _simulate_track(t, x_ref, y_ref, z_ref, score=payload["twin_score"], match_penalty=match_penalty, rng=rng, variant="twin")

        real_csv = data_dir / f"{case.name}_synthetic_real.csv"
        twin_csv = data_dir / f"{case.name}_digital_twin.csv"
        ref_csv = data_dir / f"{case.name}_reference.csv"
        _write_track_csv(real_csv, real_track)
        _write_track_csv(twin_csv, twin_track)
        _write_track_csv(ref_csv, ref_track)

        summary = {
            "name": case.name,
            "scenario": case.scenario,
            "real_rmse_m": _rmse(real_track),
            "twin_rmse_m": _rmse(twin_track),
            "real_score": payload["real_score"],
            "twin_score": payload["twin_score"],
            "match_score": payload["match"]["score"],
            "csv": {
                "reference": str(ref_csv),
                "synthetic_real": str(real_csv),
                "digital_twin": str(twin_csv),
            },
        }
        trajectory_summaries.append(summary)
        _plot_overlay(figures_dir / f"{case.name}_overlay.png", case, ref_track, real_track, twin_track, summary)

    _plot_trajectory_match_scores(figures_dir / "trajectory_match_scores.png", trajectory_summaries)

    surface_summaries: dict[str, Any] = {}
    line_summaries: dict[str, Any] = {}

    payload_values = np.linspace(0.0, 0.9, grid_points)
    z_values = np.linspace(-0.12, 0.12, grid_points)
    X, Y, match_surface, penalty_surface = _grid_scores(
        reference_metrics,
        candidate_metrics,
        payload_values,
        z_values,
        lambda payload_mass, com_z: {"payload_mass": payload_mass, "com_z": com_z},
    )
    _write_surface_csv(data_dir / "payload_z_surface.csv", X, Y, match_surface, penalty_surface)
    _plot_surface(figures_dir / "payload_z_surface.png", "Payload mass vs Z-axis payload location", "Payload mass [kg]", "Payload z-offset [m]", X, Y, match_surface, penalty_surface)
    _plot_line_family(figures_dir / "payload_z_lines.png", "Twin similarity slices for payload mass / z-offset", "Payload mass [kg]", payload_values, z_values, match_surface, lambda v: f"z = {v:+.03f} m")
    surface_summaries["payload_z"] = {"mean_similarity": float(np.mean(match_surface)), "mean_tracking_penalty_pct": float(np.mean(penalty_surface)), "csv": str(data_dir / "payload_z_surface.csv")}
    line_summaries["payload_z"] = {"figure": str(figures_dir / "payload_z_lines.png")}

    x_offsets = np.linspace(-0.12, 0.12, grid_points)
    X, Y, match_surface, penalty_surface = _grid_scores(
        reference_metrics,
        candidate_metrics,
        payload_values,
        x_offsets,
        lambda payload_mass, com_x: {"payload_mass": payload_mass, "com_x": com_x, "com_z": 0.0},
    )
    _write_surface_csv(data_dir / "payload_x_offset_surface.csv", X, Y, match_surface, penalty_surface)
    _plot_surface(figures_dir / "payload_x_offset_surface.png", "Payload mass vs X-axis payload offset", "Payload mass [kg]", "Payload x-offset [m]", X, Y, match_surface, penalty_surface)
    _plot_line_family(figures_dir / "payload_x_offset_lines.png", "Twin similarity slices for payload mass / x-offset", "Payload mass [kg]", payload_values, x_offsets, match_surface, lambda v: f"x = {v:+.03f} m")
    surface_summaries["payload_x_offset"] = {"mean_similarity": float(np.mean(match_surface)), "mean_tracking_penalty_pct": float(np.mean(penalty_surface)), "csv": str(data_dir / "payload_x_offset_surface.csv")}
    line_summaries["payload_x_offset"] = {"figure": str(figures_dir / "payload_x_offset_lines.png")}

    y_offsets = np.linspace(-0.12, 0.12, grid_points)
    X, Y, match_surface, penalty_surface = _grid_scores(
        reference_metrics,
        candidate_metrics,
        payload_values,
        y_offsets,
        lambda payload_mass, com_y: {"payload_mass": payload_mass, "com_y": com_y, "com_z": 0.0},
    )
    _write_surface_csv(data_dir / "payload_y_offset_surface.csv", X, Y, match_surface, penalty_surface)
    _plot_surface(figures_dir / "payload_y_offset_surface.png", "Payload mass vs Y-axis payload offset", "Payload mass [kg]", "Payload y-offset [m]", X, Y, match_surface, penalty_surface)
    _plot_line_family(figures_dir / "payload_y_offset_lines.png", "Twin similarity slices for payload mass / y-offset", "Payload mass [kg]", payload_values, y_offsets, match_surface, lambda v: f"y = {v:+.03f} m")
    surface_summaries["payload_y_offset"] = {"mean_similarity": float(np.mean(match_surface)), "mean_tracking_penalty_pct": float(np.mean(penalty_surface)), "csv": str(data_dir / "payload_y_offset_surface.csv")}
    line_summaries["payload_y_offset"] = {"figure": str(figures_dir / "payload_y_offset_lines.png")}

    arm_scale_values = np.linspace(0.90, 1.10, grid_points)
    X, Y, match_surface, penalty_surface = _grid_scores(
        reference_metrics,
        candidate_metrics,
        payload_values,
        arm_scale_values,
        lambda payload_mass, arm_scale: {"payload_mass": payload_mass, "arm_scale": arm_scale},
    )
    _write_surface_csv(data_dir / "arm_length_surface.csv", X, Y, match_surface, penalty_surface)
    _plot_surface(figures_dir / "arm_length_surface.png", "Payload mass vs arm length scale", "Payload mass [kg]", "Arm length scale", X, Y, match_surface, penalty_surface)
    _plot_line_family(figures_dir / "arm_length_lines.png", "Twin similarity slices for payload mass / arm scale", "Payload mass [kg]", payload_values, arm_scale_values, match_surface, lambda v: f"arm scale = {v:.02f}")
    surface_summaries["arm_length"] = {"mean_similarity": float(np.mean(match_surface)), "mean_tracking_penalty_pct": float(np.mean(penalty_surface)), "csv": str(data_dir / "arm_length_surface.csv")}
    line_summaries["arm_length"] = {"figure": str(figures_dir / "arm_length_lines.png")}

    motor_constant_scale_values = np.linspace(0.90, 1.10, grid_points)
    tau_scale_values = np.linspace(0.85, 1.15, grid_points)
    X, Y, match_surface, penalty_surface = _grid_scores(
        reference_metrics,
        candidate_metrics,
        motor_constant_scale_values,
        tau_scale_values,
        lambda motor_scale, tau_scale: {"motor_constant_scale": motor_scale, "tau_scale": tau_scale},
    )
    _write_surface_csv(data_dir / "motor_model_surface.csv", X, Y, match_surface, penalty_surface)
    _plot_surface(figures_dir / "motor_model_surface.png", "Motor constant vs motor time-constant scale", "Motor constant scale", "Motor time-scale", X, Y, match_surface, penalty_surface)
    _plot_line_family(figures_dir / "motor_model_lines.png", "Twin similarity slices for motor/time-constant mismatch", "Motor constant scale", motor_constant_scale_values, tau_scale_values, match_surface, lambda v: f"tau scale = {v:.02f}")
    surface_summaries["motor_model"] = {"mean_similarity": float(np.mean(match_surface)), "mean_tracking_penalty_pct": float(np.mean(penalty_surface)), "csv": str(data_dir / "motor_model_surface.csv")}
    line_summaries["motor_model"] = {"figure": str(figures_dir / "motor_model_lines.png")}

    summary = {
        "candidate_json": str(Path(candidate_json).resolve()) if candidate_json else "builtin:x500_family_composite_v2",
        "base_blended_twin_score": base_match["score"],
        "base_family_scores": base_match["family_scores"],
        "stage_1_real_flight_validation": {
            "note": "Synthetic noisy flight data generated from the reference x500 model. Replace the CSV files with real-flight logs later and regenerate the figures.",
            "trajectory_overlays": trajectory_summaries,
            "summary_figure": str(figures_dir / "trajectory_match_scores.png"),
        },
        "stage_2_sitl_statistical_validation": {
            "note": "Stress-test figures keep the identified twin fixed and perturb the reference plant across payload, center-of-mass, arm-length, and motor-model variations. The surface height is the twin-similarity score, and the line plots show representative slices through each surface.",
            "surface_summaries": surface_summaries,
            "line_plot_summaries": line_summaries,
        },
        "base_model_fit_figures": {
            "parameter_error_bars": str(figures_dir / "parameter_error_bars.png"),
            "family_score_bars": str(figures_dir / "family_score_bars.png"),
        },
    }
    (out_dir / "paper_validation_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate paper-ready digital-twin validation figures and summary files.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--candidate-json", default="", help="Path to identified_parameters.json or its containing directory.")
    ap.add_argument("--samples-per-traj", type=int, default=600)
    ap.add_argument("--grid-points", type=int, default=10)
    ap.add_argument("--seed", type=int, default=17)
    args = ap.parse_args()

    summary = generate_paper_artifacts(
        args.out_dir,
        candidate_json=args.candidate_json or None,
        samples_per_traj=max(120, args.samples_per_traj),
        grid_points=max(4, args.grid_points),
        seed=args.seed,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
