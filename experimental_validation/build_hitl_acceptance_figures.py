#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Build summary PNG figures for repeated HIL acceptance campaigns.")
    ap.add_argument("--run-root", required=True, help="Acceptance run root containing trajectory_runs.csv and ident_runs.csv")
    ap.add_argument("--out-dir", required=True, help="Directory for PNG outputs")
    return ap.parse_args()


def _success_rate(successes: pd.Series) -> float:
    if len(successes) == 0:
        return 0.0
    return float(successes.astype(bool).mean())


def _plot_metric_panel(ax, df: pd.DataFrame, label_col: str, metric: str, title: str, ylabel: str) -> None:
    labels = list(df[label_col].drop_duplicates())
    means = []
    stds = []
    xs = np.arange(len(labels), dtype=float)

    for idx, label in enumerate(labels):
        values = pd.to_numeric(df.loc[df[label_col] == label, metric], errors="coerce").dropna()
        means.append(float(values.mean()) if len(values) else np.nan)
        stds.append(float(values.std(ddof=0)) if len(values) > 1 else 0.0)
        jitter = np.linspace(-0.12, 0.12, max(len(values), 1))
        if len(values):
            ax.scatter(np.full(len(values), xs[idx]) + jitter[: len(values)], values, s=22, alpha=0.55, color="#355070")

    ax.errorbar(xs, means, yerr=stds, fmt="o", color="#d1495b", ecolor="#d1495b", elinewidth=2, capsize=6)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.grid(True, alpha=0.25)


def _plot_success_panel(ax, df: pd.DataFrame, label_col: str, title: str) -> None:
    labels = list(df[label_col].drop_duplicates())
    xs = np.arange(len(labels), dtype=float)
    rates = [_success_rate(df.loc[df[label_col] == label, "success"]) for label in labels]
    ax.bar(xs, rates, color="#2a9d8f", alpha=0.85)
    ax.set_ylim(0.0, 1.05)
    ax.set_title(title)
    ax.set_ylabel("Success Rate")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.grid(True, axis="y", alpha=0.25)


def build_trajectory_figure(csv_path: Path, out_path: Path) -> dict[str, object]:
    df = pd.read_csv(csv_path)
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    fig.suptitle("HITL Trajectory Acceptance Summary", fontsize=16)

    _plot_success_panel(axes[0, 0], df, "name", "Trajectory Success Rate")
    _plot_metric_panel(axes[0, 1], df, "name", "rmse_m", "RMSE by Trajectory", "RMSE [m]")
    _plot_metric_panel(axes[1, 0], df, "name", "max_z_err", "Max Z Error by Trajectory", "Max Z Error [m]")
    _plot_metric_panel(axes[1, 1], df, "name", "max_tilt", "Max Tilt by Trajectory", "Max Tilt [deg]")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)

    summary = (
        df.groupby("name", dropna=False)
        .agg(
            runs=("success", "size"),
            successes=("success", "sum"),
            rmse_mean=("rmse_m", "mean"),
            rmse_std=("rmse_m", lambda s: float(pd.Series(s).std(ddof=0) if len(s.dropna()) > 1 else 0.0)),
            max_z_mean=("max_z_err", "mean"),
            max_tilt_mean=("max_tilt", "mean"),
        )
        .reset_index()
        .to_dict(orient="records")
    )
    return {"figure": str(out_path), "summary": summary}


def build_ident_figure(csv_path: Path, out_path: Path) -> dict[str, object]:
    df = pd.read_csv(csv_path)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    fig.suptitle("HITL Identification Acceptance Summary", fontsize=16)

    _plot_success_panel(axes[0, 0], df, "profile", "Identification Success Rate")
    _plot_metric_panel(axes[0, 1], df, "profile", "max_xy", "Max XY Error by Profile", "Max XY Error [m]")
    _plot_metric_panel(axes[1, 0], df, "profile", "max_z_err", "Max Z Error by Profile", "Max Z Error [m]")
    _plot_metric_panel(axes[1, 1], df, "profile", "max_tilt", "Max Tilt by Profile", "Max Tilt [deg]")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)

    summary = (
        df.groupby("profile", dropna=False)
        .agg(
            runs=("success", "size"),
            successes=("success", "sum"),
            max_xy_mean=("max_xy", "mean"),
            max_z_mean=("max_z_err", "mean"),
            max_tilt_mean=("max_tilt", "mean"),
            duration_mean=("duration_s", "mean"),
        )
        .reset_index()
        .to_dict(orient="records")
    )
    return {"figure": str(out_path), "summary": summary}


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    payload: dict[str, object] = {"run_root": str(run_root)}

    traj_csv = run_root / "trajectory_runs.csv"
    if traj_csv.exists():
        payload["trajectory"] = build_trajectory_figure(traj_csv, out_dir / "trajectory_acceptance_summary.png")

    ident_csv = run_root / "ident_runs.csv"
    if ident_csv.exists():
        payload["ident"] = build_ident_figure(ident_csv, out_dir / "ident_acceptance_summary.png")

    (out_dir / "acceptance_figures_summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
