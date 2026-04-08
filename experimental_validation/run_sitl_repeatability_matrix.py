#!/usr/bin/env python3
"""Run repeated SITL trajectory validation and publish aggregate review CSVs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
MPLCONFIGDIR = REPO_ROOT / ".cache" / "matplotlib"
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.collect_sitl_tracking_dataset import collect_tracking_dataset
from experimental_validation.prepare_identified_model import prepare_identified_model
from experimental_validation.run_sitl_validation import ValidationModelSpec
from experimental_validation.trajectory_comparison_figures import (
    _interp_curve_at_progress,
    _load_tracking,
    _normalized_arclength,
    _trajectory_rmse,
    _trim_dataset,
)
from experimental_validation.validation_trajectories import DEFAULT_VALIDATION_TRAJECTORIES


@dataclass(frozen=True)
class RepeatModel:
    key: str
    label: str
    spec: ValidationModelSpec
    candidate_dir: Path
    prepare_model_name: str | None = None


def _prepared_model_exists(px4_root: Path, model_name: str) -> bool:
    model_sdf = px4_root / "Tools" / "simulation" / "gz" / "models" / model_name / "model.sdf"
    return model_sdf.exists()


def _resolve_trajectories(raw_names: str) -> tuple:
    if not raw_names.strip():
        return DEFAULT_VALIDATION_TRAJECTORIES
    wanted = {item.strip() for item in raw_names.split(",") if item.strip()}
    known = {entry.name: entry for entry in DEFAULT_VALIDATION_TRAJECTORIES}
    missing = sorted(wanted - set(known))
    if missing:
        raise ValueError(f"unknown trajectory name(s): {', '.join(missing)}")
    return tuple(known[name] for name in [entry.name for entry in DEFAULT_VALIDATION_TRAJECTORIES if entry.name in wanted])


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _run_metrics(case_name: str, csv_path: Path) -> dict[str, float | int]:
    timestamps_s, ref, pos = _load_tracking(csv_path)
    ref, pos = _trim_dataset(case_name, timestamps_s, ref, pos)
    rmse, errors = _trajectory_rmse(ref, pos)
    return {
        "samples": int(len(errors)),
        "rmse_m": float(rmse),
        "max_error_m": float(np.max(errors)) if len(errors) else 0.0,
        "mean_error_m": float(np.mean(errors)) if len(errors) else 0.0,
    }


def _summarize(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "median": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": float(statistics.fmean(values)),
        "median": float(statistics.median(values)),
        "std": float(statistics.pstdev(values)) if len(values) > 1 else 0.0,
        "min": float(min(values)),
        "max": float(max(values)),
    }


def _load_raw(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    df = pd.read_csv(csv_path)
    timestamps_us = df["timestamp_us"].to_numpy(dtype=float)
    timestamps_us = timestamps_us - float(timestamps_us[0])
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(dtype=float)
    pos = df[["pos_x", "pos_y", "pos_z"]].to_numpy(dtype=float)
    return timestamps_us, ref, pos


def build_mean_tracking_csv(case_name: str, csv_paths: list[Path], out_path: Path, *, samples: int = 700) -> dict:
    if not csv_paths:
        raise ValueError(f"no CSV paths for {case_name}")

    query = np.linspace(0.0, 1.0, samples)
    ref_curves: list[np.ndarray] = []
    pos_curves: list[np.ndarray] = []
    durations_us: list[float] = []

    for csv_path in csv_paths:
        timestamps_us, ref, pos = _load_raw(csv_path)
        usable = min(len(timestamps_us), len(ref), len(pos))
        if usable <= 1:
            continue
        timestamps_us = timestamps_us[:usable]
        ref = ref[:usable]
        pos = pos[:usable]
        progress = _normalized_arclength(ref)
        keep = np.concatenate(([True], np.diff(progress) > 1e-9))
        if int(np.count_nonzero(keep)) <= 1:
            continue
        ref_curves.append(_interp_curve_at_progress(ref[keep], query))
        pos_curves.append(_interp_curve_at_progress(pos[keep], query))
        durations_us.append(float(timestamps_us[-1]))

    if not ref_curves or not pos_curves:
        raise ValueError(f"no usable CSV paths for {case_name}")

    ref_mean = np.mean(np.stack(ref_curves, axis=0), axis=0)
    pos_mean = np.mean(np.stack(pos_curves, axis=0), axis=0)
    duration_us = float(statistics.median(durations_us)) if durations_us else float(samples - 1)
    timestamps_us = np.linspace(0.0, duration_us, samples)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp_us", "ref_x", "ref_y", "ref_z", "pos_x", "pos_y", "pos_z", "controller"])
        for timestamp, ref_row, pos_row in zip(timestamps_us, ref_mean, pos_mean):
            writer.writerow(
                [
                    int(round(timestamp)),
                    f"{ref_row[0]:.6f}",
                    f"{ref_row[1]:.6f}",
                    f"{ref_row[2]:.6f}",
                    f"{pos_row[0]:.6f}",
                    f"{pos_row[1]:.6f}",
                    f"{pos_row[2]:.6f}",
                    "repeat_mean",
                ]
            )
    return {"path": str(out_path.resolve()), "samples": samples, "source_count": len(csv_paths)}


def publish_local_review_sources(*, aggregate_sources: Path, docs_sources: Path) -> None:
    mapping = {
        "stock": "stock",
        "jmavsim_prior_sdf": "jmavsim_prior_sdf",
        "re_identified_from_sitl_ident": "re_identified_from_sitl_ident",
    }
    docs_sources.mkdir(parents=True, exist_ok=True)
    for src_key, dst_key in mapping.items():
        src = aggregate_sources / src_key
        if src.exists():
            src_tracking = src / "tracking_logs"
            if not src_tracking.exists():
                continue
            dst_tracking = docs_sources / dst_key / "tracking_logs"
            dst_tracking.mkdir(parents=True, exist_ok=True)
            for csv_path in src_tracking.glob("*.csv"):
                shutil.copy2(csv_path, dst_tracking / csv_path.name)


def _write_summary_markdown(summary: dict, path: Path) -> None:
    lines = [
        "# SITL 10-repeat trajectory matrix",
        "",
        f"Repetitions per model/trajectory: `{summary['repetitions']}`",
        "",
        "| Trajectory | Model | RMSE mean [m] | RMSE median [m] | RMSE std [m] | RMSE min..max [m] | Bitwise identical |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for case_name, model_rows in summary["cases"].items():
        for model_key, payload in model_rows.items():
            rmse = payload["rmse_m"]
            lines.append(
                "| "
                f"{case_name} | {payload['label']} | "
                f"{rmse['mean']:.4f} | {rmse['median']:.4f} | {rmse['std']:.4f} | "
                f"{rmse['min']:.4f}..{rmse['max']:.4f} | "
                f"{payload['bitwise_identical']} |"
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_summary_outputs(summary: dict, out_root: Path) -> None:
    (out_root / "repeatability_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_summary_markdown(summary, out_root / "repeatability_summary.md")


def run_repeatability_matrix(
    *,
    px4_root: Path,
    out_root: Path,
    prior_candidate_dir: Path,
    reidentified_candidate_dir: Path,
    repetitions: int,
    repeat_attempts: int,
    trajectories: tuple,
    publish_sources: bool,
) -> dict:
    out_root.mkdir(parents=True, exist_ok=True)

    if not _prepared_model_exists(px4_root, "x500_repeat_jmavsim_prior"):
        prepare_identified_model(px4_root, prior_candidate_dir, model_name="x500_repeat_jmavsim_prior")
    if not _prepared_model_exists(px4_root, "x500_repeat_reidentified"):
        prepare_identified_model(px4_root, reidentified_candidate_dir, model_name="x500_repeat_reidentified")

    models = (
        RepeatModel(
            key="stock",
            label="Stock x500 SITL",
            spec=ValidationModelSpec("stock_sitl_placeholder", "x500", "Stock x500 SITL"),
            candidate_dir=prior_candidate_dir,
        ),
        RepeatModel(
            key="jmavsim_prior_sdf",
            label="jMAVSim prior SDF",
            spec=ValidationModelSpec("jmavsim_prior_repeat", "x500_repeat_jmavsim_prior", "jMAVSim prior SDF"),
            candidate_dir=prior_candidate_dir,
        ),
        RepeatModel(
            key="re_identified_from_sitl_ident",
            label="Re-identified from SITL ident",
            spec=ValidationModelSpec("reidentified_repeat", "x500_repeat_reidentified", "Re-identified from SITL ident"),
            candidate_dir=reidentified_candidate_dir,
        ),
    )

    runs_root = out_root / "runs"
    aggregate_sources = out_root / "aggregate_sources"
    summary: dict = {
        "px4_root": str(px4_root),
        "repetitions": repetitions,
        "aggregate_sources": str(aggregate_sources.resolve()),
        "cases": {},
    }

    for entry in trajectories:
        case_summary: dict = {}
        for model in models:
            csv_paths: list[Path] = []
            run_metrics: list[dict] = []
            hashes: list[str] = []
            for repeat_idx in range(1, repetitions + 1):
                repeat_root = runs_root / model.key / entry.name / f"repeat_{repeat_idx:02d}"
                dst = out_root / "raw_repeats" / model.key / "tracking_logs" / entry.name / f"repeat_{repeat_idx:02d}.csv"

                reused = False
                if dst.exists() and dst.stat().st_size > 0:
                    try:
                        run_metric = _run_metrics(entry.name, dst)
                        reused = True
                        print(f"[resume] {model.key}/{entry.name}/repeat_{repeat_idx:02d}: using {dst}", flush=True)
                    except Exception as exc:
                        print(
                            f"[resume] {model.key}/{entry.name}/repeat_{repeat_idx:02d}: "
                            f"existing CSV is unusable ({exc}); rerunning",
                            flush=True,
                        )
                        dst.unlink(missing_ok=True)

                if not reused:
                    last_case: dict | None = None
                    for run_attempt in range(1, repeat_attempts + 1):
                        print(
                            f"[run] {model.key}/{entry.name}/repeat_{repeat_idx:02d} "
                            f"(attempt {run_attempt}/{repeat_attempts})",
                            flush=True,
                        )
                        dataset = collect_tracking_dataset(
                            px4_root=px4_root,
                            out_root=repeat_root,
                            candidate_dir=model.candidate_dir,
                            model_spec=model.spec,
                            trajectories=(entry,),
                            headless=True,
                            show_console=False,
                            skip_landing_after_trajectories=True,
                        )
                        case = dataset["cases"][0]
                        last_case = case
                        if case.get("tracking_log"):
                            src = Path(str(case["tracking_log"]))
                            dst.parent.mkdir(parents=True, exist_ok=True)
                            shutil.copy2(src, dst)
                            run_metric = _run_metrics(entry.name, dst)
                            break
                        if run_attempt < repeat_attempts:
                            print(
                                f"[retry] {model.key}/{entry.name}/repeat_{repeat_idx:02d}: "
                                f"no tracking log ({case.get('error', 'unknown error')})",
                                flush=True,
                            )
                    else:
                        raise RuntimeError(
                            f"missing tracking log for {model.key}/{entry.name}/repeat_{repeat_idx:02d}: {last_case}"
                        )

                csv_paths.append(dst)
                run_metric.update({"repeat": repeat_idx, "csv": str(dst.resolve()), "ok": True, "reused": reused})
                run_metrics.append(run_metric)
                hashes.append(_sha256(dst))

            aggregate_csv = aggregate_sources / model.key / "tracking_logs" / f"{entry.name}.csv"
            aggregate = build_mean_tracking_csv(entry.name, csv_paths, aggregate_csv)
            rmse_values = [float(item["rmse_m"]) for item in run_metrics]
            case_summary[model.key] = {
                "label": model.label,
                "aggregate_csv": str(aggregate_csv.resolve()),
                "aggregate": aggregate,
                "rmse_m": _summarize(rmse_values),
                "max_error_m": _summarize([float(item["max_error_m"]) for item in run_metrics]),
                "runs": run_metrics,
                "bitwise_identical": len(set(hashes)) == 1,
            }
            summary["cases"][entry.name] = case_summary
            _write_summary_outputs(summary, out_root)

    _write_summary_outputs(summary, out_root)
    if publish_sources:
        publish_local_review_sources(
            aggregate_sources=aggregate_sources,
            docs_sources=REPO_ROOT / "docs" / "sitl_validation" / "_generated" / "sources",
        )
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Run repeated SITL trajectory validation and publish aggregate review CSVs.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--out-root", default="docs/sitl_validation/repeatability_matrix")
    ap.add_argument("--prior-candidate-dir", default="examples/paper_assets/candidates/jmavsim_prior_v1")
    ap.add_argument("--reidentified-candidate-dir", default="docs/sitl_validation/_generated/candidates/re_identified_from_sitl_ident")
    ap.add_argument("--repetitions", type=int, default=10)
    ap.add_argument("--repeat-attempts", type=int, default=3, help="Retry count for one repeat if no tracking CSV is produced.")
    ap.add_argument("--trajectory-names", default="", help="Comma-separated subset. Default: all five.")
    ap.add_argument("--no-publish-local-review-sources", action="store_true")
    args = ap.parse_args()

    summary = run_repeatability_matrix(
        px4_root=Path(args.px4_root).expanduser().resolve(),
        out_root=Path(args.out_root).expanduser().resolve(),
        prior_candidate_dir=Path(args.prior_candidate_dir).expanduser().resolve(),
        reidentified_candidate_dir=Path(args.reidentified_candidate_dir).expanduser().resolve(),
        repetitions=args.repetitions,
        repeat_attempts=args.repeat_attempts,
        trajectories=_resolve_trajectories(args.trajectory_names),
        publish_sources=not args.no_publish_local_review_sources,
    )
    print(json.dumps({"ok": True, "summary": str((Path(args.out_root).expanduser().resolve() / "repeatability_summary.json"))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
