#!/usr/bin/env python3
"""Build repeatability tube summaries and figures from multi-run SITL tracking logs."""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MPLCONFIGDIR = REPO_ROOT / ".cache" / "matplotlib"
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

from experimental_validation.trajectory_comparison_figures import (
    _align_to_reference_start,
    _interp_curve_at_progress,
    _load_tracking,
    _normalized_arclength,
    _trim_dataset,
)

QUERY_SAMPLES = 700
MODEL_COLORS = {
    "stock": "#1f77b4",
    "stock_sdf_exact": "#2ca02c",
    "stock_sdf_minus100g": "#66a61e",
    "jmavsim_prior_sdf": "#d95f02",
    "jmavsim_prior_plus100g_sdf": "#e6550d",
    "re_identified_from_sitl_ident": "#6a3d9a",
    "re_identified_from_sitl_ident_plus100g": "#e7298a",
}
MODEL_LABELS = {
    "stock": "Stock x500 SITL",
    "stock_sdf_exact": "Stock SDF exact",
    "stock_sdf_minus100g": "Stock SDF -100g",
    "jmavsim_prior_sdf": "jMAVSim prior SDF",
    "jmavsim_prior_plus100g_sdf": "jMAVSim prior +100g SDF",
    "re_identified_from_sitl_ident": "Re-identified from SITL ident",
    "re_identified_from_sitl_ident_plus100g": "Re-identified from +100g SITL ident",
}
REFERENCE_COLOR = "#222222"


def _safe_float_list(values: np.ndarray, *, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values.tolist()]


def _model_color(model_key: str) -> str:
    palette = ["#1f77b4", "#2ca02c", "#66a61e", "#d95f02", "#e6550d", "#6a3d9a", "#e7298a", "#17becf", "#8c564b"]
    if model_key in MODEL_COLORS:
        return MODEL_COLORS[model_key]
    return palette[abs(hash(model_key)) % len(palette)]


def _load_repeat_curve(case_name: str, csv_path: Path, query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    timestamps_s, ref, pos = _load_tracking(csv_path)
    ref_trimmed, pos_trimmed = _trim_dataset(case_name, timestamps_s, ref, pos)
    ref_aligned, pos_aligned = _align_to_reference_start(ref_trimmed, pos_trimmed)
    progress = _normalized_arclength(ref_aligned)
    keep = np.concatenate(([True], np.diff(progress) > 1e-9))
    if int(np.count_nonzero(keep)) <= 1:
        raise ValueError(f"{csv_path} has insufficient progress variation")
    return (
        _interp_curve_at_progress(ref_aligned[keep], query),
        _interp_curve_at_progress(pos_aligned[keep], query),
    )


def _circle_intersection_area(r1: float, r2: float, d: float) -> float:
    r1 = max(float(r1), 0.0)
    r2 = max(float(r2), 0.0)
    d = max(float(d), 0.0)
    if r1 <= 1e-12 and r2 <= 1e-12:
        return 0.0
    if d >= r1 + r2:
        return 0.0
    if d <= abs(r1 - r2):
        return math.pi * min(r1, r2) ** 2
    if d <= 1e-12:
        return math.pi * min(r1, r2) ** 2
    term1 = r1 * r1 * math.acos(max(-1.0, min(1.0, (d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1))))
    term2 = r2 * r2 * math.acos(max(-1.0, min(1.0, (d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2))))
    term3 = 0.5 * math.sqrt(
        max(
            0.0,
            (-d + r1 + r2)
            * (d + r1 - r2)
            * (d - r1 + r2)
            * (d + r1 + r2),
        )
    )
    return term1 + term2 - term3


def _circle_iou(r1: float, r2: float, d: float) -> float:
    area1 = math.pi * max(r1, 0.0) ** 2
    area2 = math.pi * max(r2, 0.0) ** 2
    union = area1 + area2
    if union <= 1e-12:
        return 1.0 if d <= 1e-9 else 0.0
    inter = _circle_intersection_area(r1, r2, d)
    denom = union - inter
    if denom <= 1e-12:
        return 1.0
    return inter / denom


def _build_normals(center_xy: np.ndarray) -> np.ndarray:
    if len(center_xy) == 1:
        return np.array([[0.0, 1.0]], dtype=float)
    tangent = np.gradient(center_xy, axis=0)
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    lengths = np.linalg.norm(normals, axis=1)
    fallback = np.array([0.0, 1.0], dtype=float)
    for idx in range(len(normals)):
        if lengths[idx] > 1e-9:
            normals[idx] = normals[idx] / lengths[idx]
        elif idx > 0:
            normals[idx] = normals[idx - 1]
        else:
            normals[idx] = fallback
    return normals


def _tube_polygon(center_xy: np.ndarray, radius_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    normals = _build_normals(center_xy)
    upper = center_xy + normals * radius_xy[:, None]
    lower = center_xy - normals * radius_xy[:, None]
    polygon = np.vstack((upper, lower[::-1], upper[:1]))
    return upper, lower, polygon


def _coverage_keep_mask(sample_counts: np.ndarray, *, keep_fraction: float = 0.85) -> np.ndarray:
    if sample_counts.size == 0:
        return np.zeros((0,), dtype=bool)
    median_count = float(np.median(sample_counts))
    threshold = max(2.0, median_count * keep_fraction)
    return sample_counts >= threshold


def _endpoint_keep_mask(pos_curves: np.ndarray, base_mask: np.ndarray) -> tuple[np.ndarray, float]:
    if pos_curves.size == 0 or not np.any(base_mask):
        return np.zeros((len(pos_curves),), dtype=bool), 0.0

    endpoints = pos_curves[:, -1, :2]
    kept_endpoints = endpoints[base_mask]
    median_endpoint = np.median(kept_endpoints, axis=0)
    distances = np.linalg.norm(endpoints - median_endpoint[None, :], axis=1)
    kept_distances = distances[base_mask]
    median_distance = float(np.median(kept_distances))
    mad = float(np.median(np.abs(kept_distances - median_distance)))
    threshold = max(0.35, median_distance + 4.0 * max(mad, 1e-6))
    return distances <= threshold, threshold


def _build_model_payload(case_name: str, csv_paths: list[Path], query: np.ndarray, model_key: str, label: str) -> dict[str, object]:
    ref_curves: list[np.ndarray] = []
    pos_curves: list[np.ndarray] = []
    sample_counts: list[int] = []

    for csv_path in csv_paths:
        ref_curve, pos_curve = _load_repeat_curve(case_name, csv_path, query)
        ref_curves.append(ref_curve)
        pos_curves.append(pos_curve)
        sample_counts.append(len(_load_tracking(csv_path)[0]))

    ref_stack_all = np.stack(ref_curves, axis=0)
    pos_stack_all = np.stack(pos_curves, axis=0)
    sample_counts_arr = np.asarray(sample_counts, dtype=float)
    coverage_mask = _coverage_keep_mask(sample_counts_arr)
    endpoint_mask, endpoint_threshold = _endpoint_keep_mask(pos_stack_all, coverage_mask)
    keep_mask = coverage_mask & endpoint_mask
    if not np.any(keep_mask):
        keep_mask = coverage_mask if np.any(coverage_mask) else np.ones((len(csv_paths),), dtype=bool)

    excluded_runs = []
    for idx, csv_path in enumerate(csv_paths):
        if keep_mask[idx]:
            continue
        reasons = []
        if not coverage_mask[idx]:
            reasons.append(f"coverage<{0.85:.2f}x median")
        if not endpoint_mask[idx]:
            reasons.append("endpoint outlier")
        excluded_runs.append(
            {
                "repeat_index": idx + 1,
                "csv": str(csv_path.resolve()),
                "sample_count": int(sample_counts_arr[idx]),
                "reason": ", ".join(reasons) if reasons else "filtered",
            }
        )

    ref_stack = ref_stack_all[keep_mask]
    pos_stack = pos_stack_all[keep_mask]
    ref_mean = np.mean(ref_stack, axis=0)
    pos_mean = np.mean(pos_stack, axis=0)
    pos_std = np.std(pos_stack, axis=0, ddof=0)
    radius_xy = np.sqrt(np.sum(pos_std[:, :2] ** 2, axis=1))
    radius_xyz = np.sqrt(np.sum(pos_std ** 2, axis=1))
    upper, lower, polygon = _tube_polygon(pos_mean[:, :2], radius_xy)
    center_err = np.linalg.norm(pos_mean - ref_mean, axis=1)
    radial_xy = np.linalg.norm(pos_stack[:, :, :2] - pos_mean[None, :, :2], axis=2)

    return {
        "key": model_key,
        "label": label,
        "color": _model_color(model_key),
        "source_count": len(csv_paths),
        "included_source_count": int(np.count_nonzero(keep_mask)),
        "excluded_source_count": int(len(csv_paths) - np.count_nonzero(keep_mask)),
        "excluded_runs": excluded_runs,
        "coverage_filter": {
            "median_sample_count": round(float(np.median(sample_counts_arr)), 3) if len(sample_counts_arr) else 0.0,
            "keep_fraction": 0.85,
            "endpoint_threshold_m": round(float(endpoint_threshold), 6),
        },
        "progress": _safe_float_list(query, digits=5),
        "reference_centerline": {
            "x": _safe_float_list(ref_mean[:, 0]),
            "y": _safe_float_list(ref_mean[:, 1]),
            "z": _safe_float_list(ref_mean[:, 2]),
        },
        "centerline": {
            "x": _safe_float_list(pos_mean[:, 0]),
            "y": _safe_float_list(pos_mean[:, 1]),
            "z": _safe_float_list(pos_mean[:, 2]),
        },
        "std": {
            "x": _safe_float_list(pos_std[:, 0]),
            "y": _safe_float_list(pos_std[:, 1]),
            "z": _safe_float_list(pos_std[:, 2]),
            "xy_radius": _safe_float_list(radius_xy),
            "xyz_radius": _safe_float_list(radius_xyz),
            "xy_radius_mean": round(float(np.mean(radius_xy)), 6),
            "xy_radius_max": round(float(np.max(radius_xy)), 6),
            "xy_radius_std": round(float(np.std(radius_xy, ddof=0)), 6),
            "xy_run_radial_mean": round(float(np.mean(radial_xy)), 6),
        },
        "error_to_reference": {
            "mean_center_error": round(float(np.mean(center_err)), 6),
            "max_center_error": round(float(np.max(center_err)), 6),
        },
        "tube_xy": {
            "upper_x": _safe_float_list(upper[:, 0]),
            "upper_y": _safe_float_list(upper[:, 1]),
            "lower_x": _safe_float_list(lower[:, 0]),
            "lower_y": _safe_float_list(lower[:, 1]),
            "polygon_x": _safe_float_list(polygon[:, 0]),
            "polygon_y": _safe_float_list(polygon[:, 1]),
        },
    }


def _overlap_payload(
    case_name: str,
    left_key: str,
    left_payload: dict[str, object],
    right_key: str,
    right_payload: dict[str, object],
) -> dict[str, object]:
    left_center = np.column_stack(
        [
            np.asarray(left_payload["centerline"]["x"], dtype=float),
            np.asarray(left_payload["centerline"]["y"], dtype=float),
        ]
    )
    right_center = np.column_stack(
        [
            np.asarray(right_payload["centerline"]["x"], dtype=float),
            np.asarray(right_payload["centerline"]["y"], dtype=float),
        ]
    )
    progress = np.asarray(left_payload["progress"], dtype=float)
    r1 = np.asarray(left_payload["std"]["xy_radius"], dtype=float)
    r2 = np.asarray(right_payload["std"]["xy_radius"], dtype=float)
    d = np.linalg.norm(left_center - right_center, axis=1)
    iou = np.asarray([_circle_iou(a, b, dist) for a, b, dist in zip(r1, r2, d)], dtype=float)
    contact = d <= (r1 + r2)
    containment = d + np.minimum(r1, r2) <= np.maximum(r1, r2)

    if np.any(contact):
        focus_idx = int(np.argmax(iou))
    else:
        focus_idx = int(np.argmin(d))
    window_half = max(6, int(0.08 * len(progress)))
    start = max(0, focus_idx - window_half)
    stop = min(len(progress), focus_idx + window_half + 1)
    x_candidates = np.concatenate(
        [
            np.asarray(left_payload["tube_xy"]["upper_x"][start:stop], dtype=float),
            np.asarray(left_payload["tube_xy"]["lower_x"][start:stop], dtype=float),
            np.asarray(right_payload["tube_xy"]["upper_x"][start:stop], dtype=float),
            np.asarray(right_payload["tube_xy"]["lower_x"][start:stop], dtype=float),
        ]
    )
    y_candidates = np.concatenate(
        [
            np.asarray(left_payload["tube_xy"]["upper_y"][start:stop], dtype=float),
            np.asarray(left_payload["tube_xy"]["lower_y"][start:stop], dtype=float),
            np.asarray(right_payload["tube_xy"]["upper_y"][start:stop], dtype=float),
            np.asarray(right_payload["tube_xy"]["lower_y"][start:stop], dtype=float),
        ]
    )
    pad_x = max(0.25, 0.15 * float(np.max(x_candidates) - np.min(x_candidates) + 1e-6))
    pad_y = max(0.25, 0.15 * float(np.max(y_candidates) - np.min(y_candidates) + 1e-6))

    return {
        "case": case_name,
        "left_key": left_key,
        "right_key": right_key,
        "mean_iou_pct": round(float(np.mean(iou) * 100.0), 3),
        "median_iou_pct": round(float(np.median(iou) * 100.0), 3),
        "max_iou_pct": round(float(np.max(iou) * 100.0), 3),
        "contact_pct": round(float(np.mean(contact) * 100.0), 3),
        "containment_pct": round(float(np.mean(containment) * 100.0), 3),
        "mean_center_distance_m": round(float(np.mean(d)), 6),
        "max_center_distance_m": round(float(np.max(d)), 6),
        "mean_radius_sum_m": round(float(np.mean(r1 + r2)), 6),
        "focus_progress_pct": round(float(progress[focus_idx] * 100.0), 3),
        "focus_window_progress_pct": [
            round(float(progress[start] * 100.0), 3),
            round(float(progress[stop - 1] * 100.0), 3),
        ],
        "focus_bbox_xy": {
            "xmin": round(float(np.min(x_candidates) - pad_x), 6),
            "xmax": round(float(np.max(x_candidates) + pad_x), 6),
            "ymin": round(float(np.min(y_candidates) - pad_y), 6),
            "ymax": round(float(np.max(y_candidates) + pad_y), 6),
        },
        "sample_iou_pct": _safe_float_list(iou * 100.0, digits=4),
        "sample_center_distance_m": _safe_float_list(d),
    }


def _set_equal_xy(ax: plt.Axes, points: list[np.ndarray]) -> None:
    merged = np.vstack(points)
    xmin, ymin = np.min(merged[:, 0]), np.min(merged[:, 1])
    xmax, ymax = np.max(merged[:, 0]), np.max(merged[:, 1])
    center_x = 0.5 * (xmin + xmax)
    center_y = 0.5 * (ymin + ymax)
    radius = max(0.5 * max(xmax - xmin, ymax - ymin), 0.5)
    ax.set_xlim(center_x - radius, center_x + radius)
    ax.set_ylim(center_y - radius, center_y + radius)
    ax.set_aspect("equal", adjustable="box")


def _plot_model_tube(ax: plt.Axes, payload: dict[str, object], *, alpha: float = 0.19, label_suffix: str = "") -> None:
    color = payload["color"]
    polygon_x = np.asarray(payload["tube_xy"]["polygon_x"], dtype=float)
    polygon_y = np.asarray(payload["tube_xy"]["polygon_y"], dtype=float)
    center_x = np.asarray(payload["centerline"]["x"], dtype=float)
    center_y = np.asarray(payload["centerline"]["y"], dtype=float)
    ax.fill(polygon_x, polygon_y, color=color, alpha=alpha, linewidth=0.0)
    ax.plot(center_x, center_y, color=color, linewidth=2.6, label=f"{payload['label']}{label_suffix}")


def _render_case_figure(case_name: str, case_payload: dict[str, object], out_dir: Path) -> dict[str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    reference = case_payload["reference"]
    overlap = case_payload.get("highlight_pair")
    ref_x = np.asarray(reference["x"], dtype=float)
    ref_y = np.asarray(reference["y"], dtype=float)

    fig, ax = plt.subplots(figsize=(10.8, 8.2))
    ax.set_facecolor("#fffdf8")
    ax.plot(ref_x, ref_y, linestyle="--", linewidth=2.2, color=REFERENCE_COLOR, label="Reference")

    point_sets = [np.column_stack([ref_x, ref_y])]
    for payload in case_payload["models"].values():
        _plot_model_tube(ax, payload)
        point_sets.append(np.column_stack([np.asarray(payload["tube_xy"]["polygon_x"], dtype=float), np.asarray(payload["tube_xy"]["polygon_y"], dtype=float)]))

    _set_equal_xy(ax, point_sets)
    ax.set_title(f"{case_name}: 10-run mean path + 1σ corridor")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", frameon=True)

    annotation_lines = []
    if overlap:
        annotation_lines = [
            f"Prior vs Re-id overlap (mean IoU): {overlap['mean_iou_pct']:.1f}%",
            f"Contact over progress: {overlap['contact_pct']:.1f}%",
            f"Mean center distance: {overlap['mean_center_distance_m']:.3f} m",
            f"Focus window: {overlap['focus_window_progress_pct'][0]:.1f}%..{overlap['focus_window_progress_pct'][1]:.1f}%",
        ]
        ax.text(
            0.02,
            0.98,
            "\n".join(annotation_lines),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=10.5,
            bbox={"boxstyle": "round,pad=0.45", "facecolor": "#fff7ed", "edgecolor": "#fed7aa", "alpha": 0.96},
        )

        bbox = overlap["focus_bbox_xy"]
        inset = inset_axes(ax, width="38%", height="38%", loc="lower left", borderpad=1.2)
        inset.set_facecolor("#ffffff")
        inset.plot(ref_x, ref_y, linestyle="--", linewidth=1.2, color=REFERENCE_COLOR)
        for payload in case_payload["models"].values():
            _plot_model_tube(inset, payload, alpha=0.22)
        inset.set_xlim(bbox["xmin"], bbox["xmax"])
        inset.set_ylim(bbox["ymin"], bbox["ymax"])
        inset.set_aspect("equal", adjustable="box")
        inset.grid(True, alpha=0.22)
        inset.tick_params(labelsize=8)
        inset.set_title("Overlap zoom", fontsize=9)
        mark_inset(ax, inset, loc1=2, loc2=4, fc="none", ec="#7c2d12", alpha=0.65)

    fig.subplots_adjust(left=0.08, right=0.97, top=0.92, bottom=0.10)
    png_path = out_dir / f"{case_name}_tube.png"
    svg_path = out_dir / f"{case_name}_tube.svg"
    fig.savefig(png_path, dpi=360, bbox_inches="tight")
    fig.savefig(svg_path, dpi=360, bbox_inches="tight")
    plt.close(fig)
    return {"png": str(png_path.resolve()), "svg": str(svg_path.resolve())}


def _write_markdown(summary: dict[str, object], path: Path) -> None:
    lines = [
        "# SITL repeatability tube summary",
        "",
        "Centerline = progress-aligned 10-run mean trajectory.",
        "Tube radius = XY 1-sigma envelope around that mean trajectory.",
        "",
        "| Trajectory | Highlight pair | Mean IoU [%] | Contact [%] | Mean center distance [m] |",
        "|---|---|---:|---:|---:|",
    ]
    for case_name, case_payload in summary["cases"].items():
        overlap = case_payload.get("highlight_pair")
        if overlap:
            lines.append(
                f"| {case_name} | {overlap['left_key']} vs {overlap['right_key']} | {overlap['mean_iou_pct']:.2f} | {overlap['contact_pct']:.2f} | {overlap['mean_center_distance_m']:.4f} |"
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_repeatability_tube_outputs(*, summary: dict[str, object], out_root: Path, samples: int = QUERY_SAMPLES) -> dict[str, object]:
    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 360,
            "font.family": "DejaVu Serif",
            "font.size": 13,
            "axes.titlesize": 18,
            "axes.labelsize": 15,
            "legend.fontsize": 11,
        }
    )
    plt.style.use("seaborn-v0_8-whitegrid")

    query = np.linspace(0.0, 1.0, samples)
    cases_out: dict[str, object] = {}
    figure_dir = out_root / "tube_figures"
    stats_dir = out_root / "tube_stats"
    stats_dir.mkdir(parents=True, exist_ok=True)

    for case_name, case_summary in summary.get("cases", {}).items():
        models_out: dict[str, object] = {}
        reference_payload: dict[str, object] | None = None
        model_order = list(case_summary.keys())
        for model_key in model_order:
            payload = case_summary.get(model_key)
            if not payload:
                continue
            csv_paths = [Path(str(run["csv"])) for run in payload.get("runs", []) if run.get("ok")]
            if not csv_paths:
                continue
            model_payload = _build_model_payload(
                case_name,
                csv_paths,
                query,
                model_key,
                str(payload.get("label") or MODEL_LABELS.get(model_key, model_key)),
            )
            models_out[model_key] = model_payload
            if reference_payload is None:
                reference_payload = model_payload["reference_centerline"]

        if not models_out or reference_payload is None:
            continue

        case_payload: dict[str, object] = {
            "reference": reference_payload,
            "models": models_out,
        }
        pairwise = {}
        available_keys = list(models_out.keys())
        for left_idx, left_key in enumerate(available_keys):
            for right_key in available_keys[left_idx + 1:]:
                pairwise[f"{left_key}__{right_key}"] = _overlap_payload(
                    case_name,
                    left_key,
                    models_out[left_key],
                    right_key,
                    models_out[right_key],
                )
        if pairwise:
            case_payload["pairwise_overlap"] = pairwise
            preferred_pairs = (
                "jmavsim_prior_sdf__re_identified_from_sitl_ident",
                "jmavsim_prior_plus100g_sdf__re_identified_from_sitl_ident_plus100g",
            )
            highlight_key = next((key for key in preferred_pairs if key in pairwise), None)
            if highlight_key is None:
                highlight_key = max(pairwise, key=lambda key: float(pairwise[key]["mean_iou_pct"]))
            case_payload["highlight_pair"] = pairwise[highlight_key]

        figures = _render_case_figure(case_name, case_payload, figure_dir)
        case_payload["figures"] = figures
        cases_out[case_name] = case_payload
        (stats_dir / f"{case_name}.json").write_text(json.dumps(case_payload, indent=2), encoding="utf-8")

    out = {
        "generated_from": str(out_root.resolve()),
        "samples": samples,
        "cases": cases_out,
    }
    summary_json = out_root / "tube_summary.json"
    summary_md = out_root / "tube_summary.md"
    summary_json.write_text(json.dumps(out, indent=2), encoding="utf-8")
    _write_markdown(out, summary_md)
    out["summary_json"] = str(summary_json.resolve())
    out["summary_md"] = str(summary_md.resolve())
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Build repeatability tube figures from SITL 10-run summary data.")
    ap.add_argument("--summary-json", default="docs/sitl_validation/repeatability_matrix/repeatability_summary.json")
    ap.add_argument("--out-root", default="docs/sitl_validation/repeatability_matrix")
    ap.add_argument("--samples", type=int, default=QUERY_SAMPLES)
    args = ap.parse_args()

    summary = json.loads(Path(args.summary_json).expanduser().resolve().read_text(encoding="utf-8"))
    result = build_repeatability_tube_outputs(
        summary=summary,
        out_root=Path(args.out_root).expanduser().resolve(),
        samples=args.samples,
    )
    print(json.dumps({"ok": True, "tube_summary": result["summary_json"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
