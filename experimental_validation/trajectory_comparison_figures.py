#!/usr/bin/env python3
"""Build grouped 3D trajectory comparison figures for stock SITL and up to two other datasets."""

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


def _plot_error_collection(
    ax: plt.Axes,
    pos: np.ndarray,
    errors: np.ndarray,
    *,
    cmap,
    norm,
    linewidth: float,
) -> None:
    if len(pos) >= 2:
        segments = np.stack([pos[:-1], pos[1:]], axis=1)
        segment_error = 0.5 * (errors[:-1] + errors[1:])
        collection = Line3DCollection(segments, cmap=cmap, norm=norm, linewidth=linewidth)
        collection.set_array(segment_error)
        ax.add_collection3d(collection)
    elif len(pos) == 1:
        ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], c=[cmap(norm(float(errors[0])))], s=25)


def build_comparison_figures(
    *,
    stock_root: Path,
    compare_root: Path,
    compare_label: str,
    out_dir: Path,
    compare_root_2: Path | None = None,
    compare_label_2: str | None = None,
) -> dict[str, object]:
    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 450,
            "font.family": "DejaVu Serif",
            "font.size": 24,
            "axes.titlesize": 26,
            "axes.labelsize": 25,
            "legend.fontsize": 22,
            "xtick.labelsize": 19,
            "ytick.labelsize": 19,
        }
    )
    plt.style.use("seaborn-v0_8-whitegrid")

    has_compare_2 = compare_root_2 is not None and compare_label_2 is not None
    out_dir.mkdir(parents=True, exist_ok=True)
    figures: dict[str, str] = {}
    summary_cases: dict[str, dict[str, float | int | str | None]] = {}

    for figure_index, cases in enumerate(PANEL_GROUPS, start=1):
        cols = len(cases)
        fig = plt.figure(figsize=(12.2 * cols + (12.6 if has_compare_2 else 9.8), 13.2))
        gs = fig.add_gridspec(1, cols + 1, width_ratios=[1.0] * cols + ([0.88] if has_compare_2 else [0.66]))
        axes = [fig.add_subplot(gs[0, idx], projection="3d") for idx in range(cols)]

        if has_compare_2:
            side = gs[0, cols].subgridspec(
                2,
                4,
                height_ratios=[0.34, 0.66],
                width_ratios=[0.50, 0.17, 0.17, 0.17],
                hspace=0.08,
                wspace=0.70,
            )
        else:
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
        cax_compare_2 = fig.add_subplot(side[1, 3]) if has_compare_2 else None
        legend_ax.axis("off")
        spacer_ax.axis("off")

        global_max_stock_error = 1e-6
        global_max_compare_error = 1e-6
        global_max_compare_2_error = 1e-6
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

            payload: dict[str, object] = {
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
                "stock_label": "Stock x500 SITL",
                "compare_label": compare_label,
                "rmse_stock_m": rmse_stock,
                "rmse_compare_m": rmse_compare,
                "stock_samples": len(stock_ref_trimmed),
                "compare_samples": len(compare_ref_trimmed),
            }

            if has_compare_2:
                compare_2_ref_raw, compare_2_pos_raw = _load_tracking(compare_root_2 / "tracking_logs" / f"{case}.csv")
                compare_2_ref_trimmed, compare_2_pos_trimmed = _trim_dataset(compare_2_ref_raw, compare_2_pos_raw)
                rmse_compare_2, err_compare_2 = _trajectory_rmse(compare_2_ref_trimmed, compare_2_pos_trimmed)
                _, compare_2_pos_plot = _align_to_reference_start(compare_2_ref_trimmed, compare_2_pos_trimmed)

                payload["compare_2_plot"] = compare_2_pos_plot
                payload["rmse_compare_2"] = rmse_compare_2
                payload["err_compare_2"] = err_compare_2
                payload["compare_2_samples"] = len(compare_2_ref_trimmed)
                global_max_compare_2_error = max(
                    global_max_compare_2_error,
                    float(np.max(err_compare_2)) if len(err_compare_2) else 1e-6,
                )
                summary_cases[case]["compare_label_2"] = compare_label_2
                summary_cases[case]["rmse_compare_2_m"] = rmse_compare_2
                summary_cases[case]["compare_2_samples"] = len(compare_2_ref_trimmed)

            loaded[case] = payload

        norm_stock = mpl.colors.Normalize(vmin=0.0, vmax=global_max_stock_error)
        norm_compare = mpl.colors.Normalize(vmin=0.0, vmax=global_max_compare_error)
        norm_compare_2 = mpl.colors.Normalize(vmin=0.0, vmax=global_max_compare_2_error) if has_compare_2 else None
        cmap_stock = plt.get_cmap("viridis")
        cmap_compare = plt.get_cmap("plasma")
        cmap_compare_2 = plt.get_cmap("turbo") if has_compare_2 else None

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
            _plot_error_collection(ax, stock, err_stock, cmap=cmap_stock, norm=norm_stock, linewidth=2.8)
            _plot_error_collection(ax, compare, err_compare, cmap=cmap_compare, norm=norm_compare, linewidth=3.0)

            point_sets = [ref, stock, compare]
            title = f"{case}\nStock={payload['rmse_stock']:.3f} m | {compare_label}={payload['rmse_compare']:.3f} m"

            if has_compare_2:
                compare_2 = payload["compare_2_plot"]
                err_compare_2 = payload["err_compare_2"]
                _plot_error_collection(ax, compare_2, err_compare_2, cmap=cmap_compare_2, norm=norm_compare_2, linewidth=3.1)
                point_sets.append(compare_2)
                title += f" | {compare_label_2}={payload['rmse_compare_2']:.3f} m"

            all_points = np.vstack(point_sets)
            _set_axes_equal(ax, all_points)
            ax.set_title(title)
            ax.set_xlabel("X [m]", labelpad=18)
            ax.set_ylabel("Y [m]", labelpad=18)
            ax.set_zlabel("Z (up) [m]", labelpad=18)
            ax.view_init(elev=22, azim=-60)
            ax.tick_params(axis="both", which="major", labelsize=19)

        legend_handles = [
            mpl.lines.Line2D([0], [0], color="#304c89", linestyle="--", linewidth=2.2, label="Reference"),
            mpl.lines.Line2D([0], [0], color=cmap_stock(0.72), linestyle="-", linewidth=2.8, label="Stock x500 SITL"),
            mpl.lines.Line2D([0], [0], color=cmap_compare(0.76), linestyle="-", linewidth=3.0, label=compare_label),
        ]
        if has_compare_2:
            legend_handles.append(
                mpl.lines.Line2D([0], [0], color=cmap_compare_2(0.84), linestyle="-", linewidth=3.1, label=compare_label_2)
            )

        legend_ax.legend(
            handles=legend_handles,
            loc="upper left",
            frameon=True,
            borderpad=1.0,
            labelspacing=1.0,
            title="Curves",
            title_fontsize=22,
        )

        cbar_stock = fig.colorbar(mpl.cm.ScalarMappable(norm=norm_stock, cmap=cmap_stock), cax=cax_stock)
        cbar_compare = fig.colorbar(mpl.cm.ScalarMappable(norm=norm_compare, cmap=cmap_compare), cax=cax_compare)
        cbar_stock.set_label("Stock SITL error [m]", fontsize=22, labelpad=12)
        cbar_compare.set_label(f"{compare_label} error [m]", fontsize=22, labelpad=12)
        cbar_stock.ax.tick_params(labelsize=19)
        cbar_compare.ax.tick_params(labelsize=19)

        if has_compare_2 and cax_compare_2 is not None:
            cbar_compare_2 = fig.colorbar(
                mpl.cm.ScalarMappable(norm=norm_compare_2, cmap=cmap_compare_2),
                cax=cax_compare_2,
            )
            cbar_compare_2.set_label(f"{compare_label_2} error [m]", fontsize=22, labelpad=12)
            cbar_compare_2.ax.tick_params(labelsize=19)

        name = "group_1_circle_hairpin_lemniscate" if figure_index == 1 else "group_2_time_optimal_minimum_snap"
        out_path = out_dir / f"{name}.png"
        fig.savefig(out_path, bbox_inches="tight")
        plt.close(fig)
        figures[name] = str(out_path.resolve())

    summary = {
        "stock_label": "Stock x500 SITL",
        "compare_label": compare_label,
        "compare_label_2": compare_label_2 if has_compare_2 else None,
        "cases": summary_cases,
        "figures": figures,
    }
    summary_path = out_dir / "comparison_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    summary["summary_json"] = str(summary_path.resolve())
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate grouped trajectory comparison figures.")
    parser.add_argument("--stock-root", required=True, help="Root containing tracking_logs/*.csv for the stock SITL dataset.")
    parser.add_argument("--compare-root", required=True, help="Root containing tracking_logs/*.csv for the first comparison dataset.")
    parser.add_argument("--compare-label", required=True, help="Legend/title label for the first comparison dataset.")
    parser.add_argument("--compare-root-2", default="", help="Optional root containing tracking_logs/*.csv for the second comparison dataset.")
    parser.add_argument("--compare-label-2", default="", help="Legend/title label for the second comparison dataset.")
    parser.add_argument("--out-dir", required=True, help="Output directory for PNG figures and a JSON summary.")
    args = parser.parse_args()

    summary = build_comparison_figures(
        stock_root=Path(args.stock_root).expanduser().resolve(),
        compare_root=Path(args.compare_root).expanduser().resolve(),
        compare_label=args.compare_label,
        compare_root_2=Path(args.compare_root_2).expanduser().resolve() if args.compare_root_2 and args.compare_label_2 else None,
        compare_label_2=args.compare_label_2 or None,
        out_dir=Path(args.out_dir).expanduser().resolve(),
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
