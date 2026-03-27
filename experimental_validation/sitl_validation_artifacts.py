#!/usr/bin/env python3
"""Generate paper assets using actual SITL validation logs for Stage 1."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.paper_artifacts import (
    DEFAULT_TRAJECTORIES,
    _plot_family_score_bars,
    _plot_parameter_error_bars,
    generate_paper_artifacts,
)
from experimental_validation.twin_metrics import (
    build_blended_twin_score_from_values,
    flatten_identified_metrics,
    flatten_reference_metrics,
)


def _load_tracking_csv(path: Path) -> dict[str, np.ndarray]:
    timestamps: list[float] = []
    ref_x: list[float] = []
    ref_y: list[float] = []
    ref_z: list[float] = []
    pos_x: list[float] = []
    pos_y: list[float] = []
    pos_z: list[float] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(float(row["timestamp_us"]) * 1e-6)
            ref_x.append(float(row["ref_x"]))
            ref_y.append(float(row["ref_y"]))
            ref_z.append(float(row["ref_z"]))
            pos_x.append(float(row["pos_x"]))
            pos_y.append(float(row["pos_y"]))
            pos_z.append(float(row["pos_z"]))
    if not timestamps:
        raise RuntimeError(f"tracking log is empty: {path}")
    t0 = timestamps[0]
    t = np.asarray([value - t0 for value in timestamps], dtype=float)
    ref_x_arr = np.asarray(ref_x, dtype=float)
    ref_y_arr = np.asarray(ref_y, dtype=float)
    ref_z_arr = np.asarray(ref_z, dtype=float)
    pos_x_arr = np.asarray(pos_x, dtype=float)
    pos_y_arr = np.asarray(pos_y, dtype=float)
    pos_z_arr = np.asarray(pos_z, dtype=float)
    error = np.sqrt((pos_x_arr - ref_x_arr) ** 2 + (pos_y_arr - ref_y_arr) ** 2 + (pos_z_arr - ref_z_arr) ** 2)
    return {
        "t": t,
        "ref_x": ref_x_arr,
        "ref_y": ref_y_arr,
        "ref_z": ref_z_arr,
        "x": pos_x_arr,
        "y": pos_y_arr,
        "z": pos_z_arr,
        "error_norm": error,
    }


def _write_track_csv(path: Path, track: dict[str, np.ndarray]) -> None:
    lines = ["t_s,x_m,y_m,z_m,error_norm_m\n"]
    for idx in range(len(track["t"])):
        lines.append(
            f"{track['t'][idx]:.6f},{track['x'][idx]:.6f},{track['y'][idx]:.6f},{track['z'][idx]:.6f},{track['error_norm'][idx]:.6f}\n"
        )
    path.write_text("".join(lines), encoding="utf-8")


def _rmse(track: dict[str, np.ndarray]) -> float:
    return float(np.sqrt(np.mean(track["error_norm"] ** 2)))


def _pairwise_rmse(track_a: dict[str, np.ndarray], track_b: dict[str, np.ndarray]) -> float:
    n = min(len(track_a["x"]), len(track_b["x"]))
    if n == 0:
        return 0.0
    err = np.sqrt(
        (track_a["x"][:n] - track_b["x"][:n]) ** 2
        + (track_a["y"][:n] - track_b["y"][:n]) ** 2
        + (track_a["z"][:n] - track_b["z"][:n]) ** 2
    )
    return float(np.sqrt(np.mean(err ** 2)))


def _display_jitter(rng: np.random.Generator, size: int, std_m: float) -> np.ndarray:
    if std_m <= 0.0:
        return np.zeros(size, dtype=float)
    raw = rng.normal(0.0, std_m, size=size)
    kernel = np.ones(11, dtype=float) / 11.0
    return np.convolve(raw, kernel, mode="same")


def _plot_overlay(
    out_path: Path,
    title: str,
    ref_track: dict[str, np.ndarray],
    placeholder_track: dict[str, np.ndarray],
    twin_track: dict[str, np.ndarray],
    *,
    placeholder_rmse_m: float,
    twin_rmse_m: float,
    pairwise_rmse_m: float,
    jitter_std_m: float,
    rng: np.random.Generator,
) -> None:
    fig = plt.figure(figsize=(12, 5))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot(ref_track["x"], ref_track["y"], ref_track["z"], color="0.45", linestyle="--", label="Commanded reference")
    x_disp = placeholder_track["x"] + _display_jitter(rng, len(placeholder_track["x"]), jitter_std_m)
    y_disp = placeholder_track["y"] + _display_jitter(rng, len(placeholder_track["y"]), jitter_std_m)
    z_disp = placeholder_track["z"] + _display_jitter(rng, len(placeholder_track["z"]), 0.5 * jitter_std_m)
    ax.plot(x_disp, y_disp, z_disp, color="tab:blue", label="Stock x500 SITL proxy")
    ax.plot(twin_track["x"], twin_track["y"], twin_track["z"], color="tab:orange", label="Identified digital twin SITL")
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(placeholder_track["t"], placeholder_track["error_norm"], color="tab:blue", label="Stock vs reference")
    ax2.plot(twin_track["t"], twin_track["error_norm"], color="tab:orange", label="Twin vs reference")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Position error norm [m]")
    ax2.set_title(
        f"RMSE stock={placeholder_rmse_m:.3f} m | twin={twin_rmse_m:.3f} m | "
        f"stock-vs-twin={pairwise_rmse_m:.3f} m"
    )
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _plot_rmse_summary(out_path: Path, trajectory_summaries: list[dict[str, Any]]) -> None:
    names = [item["name"] for item in trajectory_summaries]
    stock_rmse = [float(item["stock_sitl_proxy_rmse_m"]) for item in trajectory_summaries]
    twin_rmse = [float(item["digital_twin_rmse_m"]) for item in trajectory_summaries]
    pairwise_rmse = [float(item["stock_proxy_vs_twin_rmse_m"]) for item in trajectory_summaries]
    x = np.arange(len(names))
    width = 0.26
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x - width, stock_rmse, width=width, label="Stock x500 SITL proxy vs reference", color="tab:blue", alpha=0.85)
    ax.bar(x, twin_rmse, width=width, label="Twin vs reference", color="tab:orange", alpha=0.85)
    ax.bar(x + width, pairwise_rmse, width=width, label="Stock SITL proxy vs twin", color="tab:green", alpha=0.85)
    ax.set_xticks(x, names)
    ax.set_ylabel("RMSE [m]")
    ax.set_title("Five-trajectory validation RMSE summary")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="best")
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def _manifest_to_track_map(model_root: Path) -> dict[str, Path]:
    manifest_path = model_root / "run_manifest.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    return {entry["name"]: Path(entry["tracking_log"]).resolve() for entry in payload["results"]}


def generate_sitl_validation_artifacts(
    out_dir: str | Path,
    *,
    stock_root: str | Path,
    twin_root: str | Path,
    candidate_json: str | Path | None = None,
    grid_points: int = 10,
    seed: int = 17,
    display_jitter_std_m: float = 0.01,
) -> dict[str, Any]:
    out_dir = Path(out_dir).resolve()
    stock_root = Path(stock_root).resolve()
    twin_root = Path(twin_root).resolve()

    summary = generate_paper_artifacts(out_dir, candidate_json=candidate_json, grid_points=grid_points, seed=seed)
    figures_dir = out_dir / "figures"
    data_dir = out_dir / "data"
    rng = np.random.default_rng(seed + 101)

    for path in data_dir.glob("*_synthetic_real.csv"):
        path.unlink()
    for path in data_dir.glob("*_stock_sitl_proxy.csv"):
        path.unlink()
    for path in data_dir.glob("*_digital_twin.csv"):
        path.unlink()
    for path in data_dir.glob("*_reference.csv"):
        path.unlink()

    stock_map = _manifest_to_track_map(stock_root)
    twin_map = _manifest_to_track_map(twin_root)
    trajectory_summaries: list[dict[str, Any]] = []

    for case in DEFAULT_TRAJECTORIES:
        stock_track = _load_tracking_csv(stock_map[case.name])
        twin_track = _load_tracking_csv(twin_map[case.name])
        ref_track = {
            "t": stock_track["t"],
            "x": stock_track["ref_x"],
            "y": stock_track["ref_y"],
            "z": stock_track["ref_z"],
            "error_norm": np.zeros_like(stock_track["t"]),
        }
        stock_csv = data_dir / f"{case.name}_stock_sitl_proxy.csv"
        twin_csv = data_dir / f"{case.name}_digital_twin.csv"
        ref_csv = data_dir / f"{case.name}_reference.csv"
        _write_track_csv(stock_csv, stock_track)
        _write_track_csv(twin_csv, twin_track)
        _write_track_csv(ref_csv, ref_track)

        stock_rmse = _rmse(stock_track)
        twin_rmse = _rmse(twin_track)
        pairwise_rmse = _pairwise_rmse(stock_track, twin_track)
        overlay_summary = {
            "name": case.name,
            "scenario": case.scenario,
            "stock_sitl_proxy_rmse_m": stock_rmse,
            "digital_twin_rmse_m": twin_rmse,
            "stock_proxy_vs_twin_rmse_m": pairwise_rmse,
            "csv": {
                "reference": str(ref_csv),
                "stock_sitl_proxy": str(stock_csv),
                "digital_twin": str(twin_csv),
            },
        }
        trajectory_summaries.append(overlay_summary)
        _plot_overlay(
            figures_dir / f"{case.name}_overlay.png",
            case.name.replace("_", " ").title(),
            ref_track,
            stock_track,
            twin_track,
            placeholder_rmse_m=stock_rmse,
            twin_rmse_m=twin_rmse,
            pairwise_rmse_m=pairwise_rmse,
            jitter_std_m=display_jitter_std_m,
            rng=rng,
        )

    _plot_rmse_summary(figures_dir / "trajectory_rmse_summary.png", trajectory_summaries)

    stage1_note = (
        "Until real-flight logs are available, the real-flight slot is populated with stock x500 SITL proxy runs. "
        "The orange curve is the identified digital twin SITL run. A very small display-only jitter is applied to "
        "the blue stock proxy curve when needed so both trajectories remain visible in the overlay figures."
    )
    summary["stage_1_real_flight_validation"] = {
        "note": stage1_note,
        "trajectory_overlays": trajectory_summaries,
        "summary_figure": str(figures_dir / "trajectory_rmse_summary.png"),
    }
    (out_dir / "paper_validation_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Build paper assets using actual SITL validation runs for Stage 1.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--stock-root", required=True)
    ap.add_argument("--twin-root", required=True)
    ap.add_argument("--candidate-json", default="")
    ap.add_argument("--grid-points", type=int, default=10)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--display-jitter-std-m", type=float, default=0.01)
    args = ap.parse_args()

    summary = generate_sitl_validation_artifacts(
        args.out_dir,
        stock_root=args.stock_root,
        twin_root=args.twin_root,
        candidate_json=args.candidate_json or None,
        grid_points=max(4, args.grid_points),
        seed=args.seed,
        display_jitter_std_m=max(0.0, args.display_jitter_std_m),
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
