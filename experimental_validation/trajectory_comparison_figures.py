#!/usr/bin/env python3
"""Build grouped trajectory comparison figures for two tracking-log datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Line3DCollection

PANEL_GROUPS: tuple[tuple[str, ...], ...] = (
    ("circle", "hairpin", "lemniscate"),
    ("time_optimal_30s", "minimum_snap_50s"),
)


def _load_tracking(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(csv_path)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(dtype=float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(dtype=float)
    ref[:, 2] *= -1.0
    pos[:, 2] *= -1.0
    return ref, pos


def _trajectory_rmse(ref: np.ndarray, pos: np.ndarray) -> tuple[float, np.ndarray]:
    usable = min(len(ref), len(pos))
    if usable == 0:
        return 0.0, np.empty((0,), dtype=float)
    delta = pos[:usable] - ref[:usable]
    sq = np.sum(delta ** 2, axis=1)
    return float(np.sqrt(np.mean(sq))), np.sqrt(sq)


def _runaway_cutoff_index(
    ref: np.ndarray,
    pos: np.ndarray,
    *,
    z_error_threshold_m: float = 6.0,
    consecutive_samples: int = 12,
) -> int:
    usable = min(len(ref), len(pos))
    if usable == 0:
        return 0

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


def _trim_dataset(ref: np.ndarray, pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cutoff = max(1, _runaway_cutoff_index(ref, pos))
    return ref[:cutoff], pos[:cutoff]


def _align_to_reference_start(ref: np.ndarray, pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(ref) == 0:
        return ref.copy(), pos.copy()
    offset = ref[0].copy()
    return ref - offset, pos - offset


def _set_axes_equal(ax: plt.Axes, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = max(float(np.max(maxs - mins)) / 2.0, 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def build_comparison_figures(
    *,
    stock_root: Path,
    compare_root: Path,
    compare_label: str,
    out_dir: Path,
) -> dict[str, object]:
    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 450,
            "font.family": "DejaVu Serif",
            "font.size": 24,
            "axes.titlesize": 28,
            "axes.labelsize": 25,
            "legend.fontsize": 23,
            "xtick.labelsize": 20,
            "ytick.labelsize": 20,
        }
    )
    plt.style.use("seaborn-v0_8-whitegrid")

    out_dir.mkdir(parents=True, exist_ok=True)
    figures: dict[str, str] = {}
    summary_cases: dict[str, dict[str, float | int | str]] = {}

    for figure_index, cases in enumerate(PANEL_GROUPS, start=1):
        cols = len(cases)
        fig = plt.figure(figsize=(12.2 * cols + 9.8, 13.2))
        gs = fig.add_gridspec(1, cols + 1, width_ratios=[1.0] * cols + [0.66])
        axes = [fig.add_subplot(gs[0, idx], projection="3d") for idx in range(cols)]
        side = gs[0, cols].subgridspec(
            2,
            3,
            height_ratios=[0.34, 0.66],
            width_ratios=[0.62, 0.19, 0.19],
            hspace=0.08,
            wspace=0.55,
        )
        legend_ax = fig.add_subplot(side[0, :])
        spacer_ax = fig.add_subplot(side[1, 0])
        cax_stock = fig.add_subplot(side[1, 1])
        cax_compare = fig.add_subplot(side[1, 2])
        legend_ax.axis("off")
        spacer_ax.axis("off")

        global_max_stock_error = 1e-6
        global_max_compare_error = 1e-6
        loaded: dict[str, dict[str, object]] = {}

        for case in cases:
            stock_ref_raw, stock_pos_raw = _load_tracking(stock_root / "tracking_logs" / f"{case}.csv")
            compare_ref_raw, compare_pos_raw = _load_tracking(compare_root / "tracking_logs" / f"{case}.csv")

            stock_ref_trimmed, stock_pos_trimmed = _trim_dataset(stock_ref_raw, stock_pos_raw)
            compare_ref_trimmed, compare_pos_trimmed = _trim_dataset(compare_ref_raw, compare_pos_raw)

            rmse_stock, err_stock = _trajectory_rmse(stock_ref_trimmed, stock_pos_trimmed)
            rmse_compare, err_compare = _trajectory_rmse(compare_ref_trimmed, compare_pos_trimmed)
            global_max_stock_error = max(global_max_stock_error, float(np.max(err_stock)) if len(err_stock) else 1e-6)
            global_max_compare_error = max(global_max_compare_error, float(np.max(err_compare)) if len(err_compare) else 1e-6)

            stock_ref_plot, stock_pos_plot = _align_to_reference_start(stock_ref_trimmed, stock_pos_trimmed)
            _, compare_pos_plot = _align_to_reference_start(compare_ref_trimmed, compare_pos_trimmed)

            loaded[case] = {
                "ref_plot": stock_ref_plot,
                "stock_plot": stock_pos_plot,
                "compare_plot": compare_pos_plot,
                "rmse_stock": rmse_stock,
                "rmse_compare": rmse_compare,
                "err_stock": err_stock,
                "err_compare": err_compare,
                "stock_samples": len(stock_ref_trimmed),
                "compare_samples": len(compare_ref_trimmed),
            }
            summary_cases[case] = {
                "stock_label": "SITL",
                "compare_label": compare_label,
                "rmse_sitl_m": rmse_stock,
                "rmse_compare_m": rmse_compare,
                "stock_samples": len(stock_ref_trimmed),
                "compare_samples": len(compare_ref_trimmed),
            }

        norm_stock = mpl.colors.Normalize(vmin=0.0, vmax=global_max_stock_error)
        norm_compare = mpl.colors.Normalize(vmin=0.0, vmax=global_max_compare_error)
        cmap_stock = plt.get_cmap("viridis")
        cmap_compare = plt.get_cmap("turbo")

        for ax, case in zip(axes, cases):
            payload = loaded[case]
            ref = payload["ref_plot"]
            stock = payload["stock_plot"]
            compare = payload["compare_plot"]
            err_stock = payload["err_stock"]
            err_compare = payload["err_compare"]

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
            elif len(stock) == 1:
                ax.scatter(stock[:, 0], stock[:, 1], stock[:, 2], c=[cmap_stock(norm_stock(float(err_stock[0])))], s=25)

            if len(compare) >= 2:
                compare_segments = np.stack([compare[:-1], compare[1:]], axis=1)
                compare_segment_error = 0.5 * (err_compare[:-1] + err_compare[1:])
                compare_lc = Line3DCollection(compare_segments, cmap=cmap_compare, norm=norm_compare, linewidth=3.0)
                compare_lc.set_array(compare_segment_error)
                ax.add_collection3d(compare_lc)
            elif len(compare) == 1:
                ax.scatter(compare[:, 0], compare[:, 1], compare[:, 2], c=[cmap_compare(norm_compare(float(err_compare[0])))], s=25)

            all_points = np.vstack([ref, stock, compare])
            _set_axes_equal(ax, all_points)
            ax.set_title(
                f"{case}\nRMSE SITL={payload['rmse_stock']:.3f} m | {compare_label}={payload['rmse_compare']:.3f} m"
            )
            ax.set_xlabel("X [m]", labelpad=18)
            ax.set_ylabel("Y [m]", labelpad=18)
            ax.set_zlabel("Z (up) [m]", labelpad=18)
            ax.view_init(elev=22, azim=-60)
            ax.tick_params(axis="both", which="major", labelsize=20)

        legend_handles = [
            mpl.lines.Line2D([0], [0], color="#304c89", linestyle="--", linewidth=2.2, label="Reference"),
            mpl.lines.Line2D([0], [0], color=cmap_stock(0.72), linestyle="-", linewidth=2.8, label="SITL"),
            mpl.lines.Line2D([0], [0], color="#d1495b", linestyle="-", linewidth=3.0, label=compare_label),
        ]
        legend_ax.legend(
            handles=legend_handles,
            loc="upper left",
            frameon=True,
            borderpad=1.0,
            labelspacing=1.0,
            title="Curves",
            title_fontsize=23,
        )
        scalar_stock = mpl.cm.ScalarMappable(norm=norm_stock, cmap=cmap_stock)
        scalar_compare = mpl.cm.ScalarMappable(norm=norm_compare, cmap=cmap_compare)
        cbar_stock = fig.colorbar(scalar_stock, cax=cax_stock)
        cbar_compare = fig.colorbar(scalar_compare, cax=cax_compare)
        cbar_stock.set_label("SITL error [m]", fontsize=22, labelpad=12)
        cbar_compare.set_label(f"{compare_label} error [m]", fontsize=22, labelpad=12)
        cbar_stock.ax.tick_params(labelsize=20)
        cbar_compare.ax.tick_params(labelsize=20)

        name = "group_1_circle_hairpin_lemniscate" if figure_index == 1 else "group_2_time_optimal_minimum_snap"
        out_path = out_dir / f"{name}.png"
        fig.savefig(out_path, bbox_inches="tight")
        plt.close(fig)
        figures[name] = str(out_path.resolve())

    summary = {
        "stock_label": "SITL",
        "compare_label": compare_label,
        "cases": summary_cases,
        "figures": figures,
    }
    summary_path = out_dir / "comparison_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    summary["summary_json"] = str(summary_path.resolve())
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate grouped trajectory comparison figures for two tracking-log datasets.")
    parser.add_argument("--stock-root", required=True, help="Root containing tracking_logs/*.csv for the stock SITL dataset.")
    parser.add_argument("--compare-root", required=True, help="Root containing tracking_logs/*.csv for the comparison dataset.")
    parser.add_argument("--compare-label", required=True, help="Legend/title label for the comparison dataset.")
    parser.add_argument("--out-dir", required=True, help="Output directory for PNG figures and a JSON summary.")
    args = parser.parse_args()

    summary = build_comparison_figures(
        stock_root=Path(args.stock_root).expanduser().resolve(),
        compare_root=Path(args.compare_root).expanduser().resolve(),
        compare_label=args.compare_label,
        out_dir=Path(args.out_dir).expanduser().resolve(),
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
