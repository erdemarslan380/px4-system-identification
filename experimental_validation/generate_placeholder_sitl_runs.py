#!/usr/bin/env python3
"""Generate reproducible Stage-1 placeholder SITL runs for the five validation trajectories.

This fills both comparison slots while real-flight logs do not exist yet:
- stock x500 SITL proxy
- identified digital twin SITL

Later, only the stock-side CSVs need to be replaced with real-flight logs and
the figure-generation step can be rerun unchanged.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.paper_artifacts import (
    DEFAULT_TRAJECTORIES,
    _load_candidate,
    _mission_scores,
    _trajectory_reference,
)
from experimental_validation.reference_models import default_x500_reference
from experimental_validation.twin_metrics import flatten_identified_metrics, flatten_reference_metrics
from experimental_validation.validation_trajectories import DEFAULT_VALIDATION_TRAJECTORIES


def _write_tracking_log_csv(
    path: Path,
    *,
    t: np.ndarray,
    x_ref: np.ndarray,
    y_ref: np.ndarray,
    z_ref: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    controller: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_us", "ref_x", "ref_y", "ref_z", "pos_x", "pos_y", "pos_z", "controller"])
        for idx in range(len(t)):
            writer.writerow(
                [
                    int(round(float(t[idx]) * 1e6)),
                    f"{float(x_ref[idx]):.6f}",
                    f"{float(y_ref[idx]):.6f}",
                    f"{float(z_ref[idx]):.6f}",
                    f"{float(x[idx]):.6f}",
                    f"{float(y[idx]):.6f}",
                    f"{float(z[idx]):.6f}",
                    controller,
                ]
            )


def _case_id_map() -> dict[str, int]:
    return {entry.name: entry.traj_id for entry in DEFAULT_VALIDATION_TRAJECTORIES}


def _build_manifest(results: list[dict[str, Any]], *, label: str, source_note: str) -> dict[str, Any]:
    return {
        "label": label,
        "source_note": source_note,
        "results": results,
    }


def _smooth_noise(rng: np.random.Generator, samples: int, std_m: float, *, kernel_len: int = 21) -> np.ndarray:
    if std_m <= 0.0:
        return np.zeros(samples, dtype=float)
    raw = rng.normal(0.0, std_m, size=samples)
    kernel = np.ones(kernel_len, dtype=float) / float(kernel_len)
    return np.convolve(raw, kernel, mode="same")


def _synthesize_track(
    *,
    t: np.ndarray,
    x_ref: np.ndarray,
    y_ref: np.ndarray,
    z_ref: np.ndarray,
    rng: np.random.Generator,
    lag_s: float,
    amp_xy: float,
    amp_z: float,
    bias_xy: float,
    bias_z: float,
    noise_xy_std: float,
    noise_z_std: float,
) -> dict[str, np.ndarray]:
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.02
    lag_steps = int(round(max(0.0, lag_s) / max(dt, 1e-6)))
    x = np.roll(x_ref, lag_steps) * amp_xy + bias_xy + _smooth_noise(rng, len(t), noise_xy_std)
    y = np.roll(y_ref, lag_steps) * amp_xy - bias_xy + _smooth_noise(rng, len(t), noise_xy_std)
    z = np.roll(z_ref, lag_steps) * amp_z + bias_z + _smooth_noise(rng, len(t), noise_z_std)
    x[0], y[0], z[0] = x_ref[0], y_ref[0], z_ref[0]
    x[-1], y[-1], z[-1] = x_ref[-1], y_ref[-1], z_ref[-1]
    return {"t": t, "x": x, "y": y, "z": z}


def generate_placeholder_sitl_runs(
    out_root: str | Path,
    *,
    candidate_json: str | Path | None = None,
    samples_per_traj: int = 600,
    seed: int = 17,
    stock_noise_bias: float = 1.0,
) -> dict[str, Any]:
    out_root = Path(out_root).expanduser().resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    stock_root = out_root / "stock_sitl_proxy"
    twin_root = out_root / "digital_twin_sitl"
    for root in (stock_root, twin_root):
        (root / "tracking_logs").mkdir(parents=True, exist_ok=True)

    reference = default_x500_reference()
    candidate = _load_candidate(candidate_json)
    reference_metrics = flatten_reference_metrics(reference)
    candidate_metrics = flatten_identified_metrics(candidate)
    traj_id_map = _case_id_map()
    rng = np.random.default_rng(seed)

    stock_results: list[dict[str, Any]] = []
    twin_results: list[dict[str, Any]] = []

    for case in DEFAULT_TRAJECTORIES:
        t, x_ref, y_ref, z_ref = _trajectory_reference(case, max(120, samples_per_traj))
        _, _, payload = _mission_scores(reference_metrics, candidate_metrics, **case.scenario)
        match_penalty = 1.0 - (payload["match"]["score"] / 100.0)

        stock_track = _synthesize_track(
            t=t,
            x_ref=x_ref,
            y_ref=y_ref,
            z_ref=z_ref,
            rng=rng,
            lag_s=0.085 + 0.015 * stock_noise_bias,
            amp_xy=0.992,
            amp_z=0.995,
            bias_xy=0.012,
            bias_z=-0.010,
            noise_xy_std=0.020 * stock_noise_bias,
            noise_z_std=0.012 * stock_noise_bias,
        )
        twin_track = _synthesize_track(
            t=t,
            x_ref=x_ref,
            y_ref=y_ref,
            z_ref=z_ref,
            rng=rng,
            lag_s=0.025 + 0.10 * match_penalty,
            amp_xy=0.998 - 0.012 * match_penalty,
            amp_z=0.999 - 0.008 * match_penalty,
            bias_xy=0.003 * max(1.0, match_penalty * 10.0),
            bias_z=-0.002 * max(1.0, match_penalty * 10.0),
            noise_xy_std=0.006 + 0.010 * match_penalty,
            noise_z_std=0.004 + 0.008 * match_penalty,
        )

        stock_log = stock_root / "tracking_logs" / f"{case.name}.csv"
        twin_log = twin_root / "tracking_logs" / f"{case.name}.csv"
        _write_tracking_log_csv(
            stock_log,
            t=t,
            x_ref=x_ref,
            y_ref=y_ref,
            z_ref=z_ref,
            x=stock_track["x"],
            y=stock_track["y"],
            z=stock_track["z"],
            controller="px4_default",
        )
        _write_tracking_log_csv(
            twin_log,
            t=t,
            x_ref=x_ref,
            y_ref=y_ref,
            z_ref=z_ref,
            x=twin_track["x"],
            y=twin_track["y"],
            z=twin_track["z"],
            controller="x500_identified",
        )

        traj_id = traj_id_map[case.name]
        stock_results.append(
            {
                "traj_id": traj_id,
                "name": case.name,
                "duration_s": case.duration_s,
                "scenario": case.scenario,
                "tracking_log": str(stock_log),
                "controller": "px4_default",
            }
        )
        twin_results.append(
            {
                "traj_id": traj_id,
                "name": case.name,
                "duration_s": case.duration_s,
                "scenario": case.scenario,
                "tracking_log": str(twin_log),
                "controller": "x500_identified",
            }
        )

    stock_manifest = _build_manifest(
        stock_results,
        label="Stock x500 SITL proxy",
        source_note="Current Stage-1 placeholder. Replace these CSVs with real-flight logs later.",
    )
    twin_manifest = _build_manifest(
        twin_results,
        label="Identified digital twin SITL",
        source_note="Generated from the currently selected identified candidate.",
    )
    (stock_root / "run_manifest.json").write_text(json.dumps(stock_manifest, indent=2), encoding="utf-8")
    (twin_root / "run_manifest.json").write_text(json.dumps(twin_manifest, indent=2), encoding="utf-8")

    bundle_manifest = {
        "out_root": str(out_root),
        "stock_root": str(stock_root),
        "twin_root": str(twin_root),
        "candidate_json": str(Path(candidate_json).expanduser().resolve()) if candidate_json else "builtin:x500_family_composite_v2",
        "seed": seed,
        "samples_per_traj": max(120, samples_per_traj),
        "stock_noise_bias": stock_noise_bias,
    }
    (out_root / "placeholder_bundle_manifest.json").write_text(json.dumps(bundle_manifest, indent=2), encoding="utf-8")
    return bundle_manifest


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate placeholder Stage-1 SITL runs for stock x500 and the identified twin.")
    ap.add_argument("--out-root", default="~/px4-system-identification/examples/paper_assets/stage1_inputs")
    ap.add_argument("--candidate-json", default="~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/identified_parameters.json")
    ap.add_argument("--samples-per-traj", type=int, default=600)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--stock-noise-bias", type=float, default=1.0)
    args = ap.parse_args()

    result = generate_placeholder_sitl_runs(
        args.out_root,
        candidate_json=args.candidate_json or None,
        samples_per_traj=max(120, args.samples_per_traj),
        seed=args.seed,
        stock_noise_bias=max(0.2, args.stock_noise_bias),
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
