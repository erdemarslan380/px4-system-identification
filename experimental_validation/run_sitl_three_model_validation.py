#!/usr/bin/env python3
"""Run the three-model SITL validation matrix and build comparison figures.

Workflow:
1. Collect the five stock x500 validation trajectories.
2. Prepare and collect the five trajectories for the jMAVSim-prior SDF.
3. Run the nine identification maneuvers on the jMAVSim-prior SDF.
4. Re-identify a new candidate from those SITL identification logs.
5. Prepare and collect the five trajectories for the re-identified SDF.
6. Generate the main comparison figures that overlay:
   - reference
   - stock x500
   - jMAVSim-prior SDF
   - re-identified SDF
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.collect_sitl_tracking_dataset import collect_tracking_dataset
from experimental_validation.compare_with_sdf import (
    build_identification_mode_reports,
    choose_primary_identification_mode,
    collect_identification_logs,
    parse_x500_sdf_reference,
    write_comparison_outputs,
)
from experimental_validation.offnominal_sitl_study import IDENT_PROFILES, run_identification_with_assets
from experimental_validation.prepare_identified_model import prepare_identified_model
from experimental_validation.run_sitl_validation import (
    DEFAULT_MODEL_SPECS,
    ValidationModelSpec,
)
from experimental_validation.trajectory_comparison_figures import build_comparison_figures

STOCK_MODEL_SPEC = DEFAULT_MODEL_SPECS[0]
PRIOR_MODEL_NAME = "x500_ident_matrix_prior"
PRIOR_MODEL_SPEC = ValidationModelSpec(
    label="jmavsim_prior_sitl",
    gz_model=PRIOR_MODEL_NAME,
    display_name="jMAVSim prior SDF",
)
REIDENT_MODEL_NAME = "x500_ident_matrix_reidentified"
REIDENT_MODEL_SPEC = ValidationModelSpec(
    label="reidentified_sitl",
    gz_model=REIDENT_MODEL_NAME,
    display_name="Re-identified from SITL ident",
)


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _has_all_tracking_logs(dataset_root: Path) -> bool:
    summary_path = dataset_root / "collection_summary.json"
    if not summary_path.exists():
        return False
    summary = _read_json(summary_path)
    cases = summary.get("cases", [])
    if len(cases) != 5:
        return False
    for case in cases:
        path = case.get("tracking_log")
        if not path or not Path(path).exists():
            return False
    return True


def _has_all_ident_logs(result_root: Path) -> bool:
    manifest_path = result_root / "run_manifest.json"
    if not manifest_path.exists():
        return False
    manifest = _read_json(manifest_path)
    results = manifest.get("results", [])
    if len(results) != len(IDENT_PROFILES):
        return False
    for entry in results:
        for key in ("identification_log", "tracking_log", "truth_log"):
            path = entry.get(key)
            if not path or not Path(path).exists():
                return False
    return True


def _collect_or_reuse_dataset(
    *,
    px4_root: Path,
    out_root: Path,
    candidate_dir: Path,
    model_spec: ValidationModelSpec,
    force: bool,
) -> dict:
    dataset_root = out_root / model_spec.label
    if not force and _has_all_tracking_logs(dataset_root):
        return _read_json(dataset_root / "collection_summary.json")
    return collect_tracking_dataset(
        px4_root=px4_root,
        out_root=out_root,
        candidate_dir=candidate_dir,
        model_spec=model_spec,
        headless=True,
        show_console=False,
    )


def _run_or_reuse_ident(
    *,
    px4_root: Path,
    out_root: Path,
    label: str,
    source_model_dir: Path,
    model_name: str,
    force: bool,
) -> dict:
    result_root = out_root / label
    if not force and _has_all_ident_logs(result_root):
        return _read_json(result_root / "run_manifest.json")
    return run_identification_with_assets(
        px4_root=px4_root,
        out_root=out_root,
        label=label,
        source_model_dir=source_model_dir,
        model_name=model_name,
        headless=True,
    )


def _build_reidentified_candidate(
    *,
    ident_root: Path,
    prepared_model_dir: Path,
    prepared_base_model_dir: Path,
    out_dir: Path,
    force: bool,
) -> dict:
    identified_json = out_dir / "identified_parameters.json"
    if not force and identified_json.exists():
        return {
            "out_dir": str(out_dir.resolve()),
            "primary_mode": _read_json(out_dir / "used_identification_logs.json")["primary_mode"],
            "identified_parameters": str(identified_json.resolve()),
        }

    csv_paths = collect_identification_logs(ident_root, latest_only=True)
    sdf_reference = parse_x500_sdf_reference(
        prepared_model_dir / "model.sdf",
        prepared_base_model_dir / "model.sdf",
    )
    reports_by_mode = build_identification_mode_reports(csv_paths, sdf_reference)
    primary_mode = choose_primary_identification_mode(reports_by_mode)
    write_comparison_outputs(
        out_dir,
        reports_by_mode=reports_by_mode,
        sdf_reference=sdf_reference,
        csv_paths=csv_paths,
        primary_mode=primary_mode,
    )
    return {
        "out_dir": str(out_dir.resolve()),
        "primary_mode": primary_mode,
        "identified_parameters": str(identified_json.resolve()),
    }


def run_three_model_validation(
    *,
    px4_root: Path,
    out_root: Path,
    prior_candidate_dir: Path,
    force: bool = False,
) -> dict:
    out_root.mkdir(parents=True, exist_ok=True)

    stock_runs_root = out_root / "stock_dataset"
    prior_runs_root = out_root / "jmavsim_prior_dataset"
    ident_runs_root = out_root / "jmavsim_prior_ident"
    reidentified_candidate_dir = out_root / "reidentified_candidate_from_sitl_ident"
    reidentified_runs_root = out_root / "reidentified_dataset"
    figures_root = out_root / "comparison_figures"

    stock_summary = _collect_or_reuse_dataset(
        px4_root=px4_root,
        out_root=stock_runs_root,
        candidate_dir=prior_candidate_dir,
        model_spec=STOCK_MODEL_SPEC,
        force=force,
    )

    prior_prepare = prepare_identified_model(px4_root, prior_candidate_dir, model_name=PRIOR_MODEL_NAME)
    prior_summary = _collect_or_reuse_dataset(
        px4_root=px4_root,
        out_root=prior_runs_root,
        candidate_dir=prior_candidate_dir,
        model_spec=PRIOR_MODEL_SPEC,
        force=force,
    )

    ident_manifest = _run_or_reuse_ident(
        px4_root=px4_root,
        out_root=ident_runs_root,
        label="jmavsim_prior_ident_suite",
        source_model_dir=Path(prior_prepare["model_dir"]),
        model_name=PRIOR_MODEL_NAME,
        force=force,
    )

    candidate_summary = _build_reidentified_candidate(
        ident_root=ident_runs_root / "jmavsim_prior_ident_suite",
        prepared_model_dir=Path(prior_prepare["model_dir"]),
        prepared_base_model_dir=Path(prior_prepare["base_model_dir"]),
        out_dir=reidentified_candidate_dir,
        force=force,
    )

    reidentified_prepare = prepare_identified_model(
        px4_root,
        reidentified_candidate_dir,
        model_name=REIDENT_MODEL_NAME,
    )
    reidentified_summary = _collect_or_reuse_dataset(
        px4_root=px4_root,
        out_root=reidentified_runs_root,
        candidate_dir=reidentified_candidate_dir,
        model_spec=REIDENT_MODEL_SPEC,
        force=force,
    )

    figures_summary = build_comparison_figures(
        stock_root=Path(stock_summary["dataset_root"]),
        compare_root=Path(prior_summary["dataset_root"]),
        compare_label="jMAVSim prior SDF",
        compare_root_2=Path(reidentified_summary["dataset_root"]),
        compare_label_2="Re-identified from SITL ident",
        out_dir=figures_root,
    )

    summary = {
        "px4_root": str(px4_root.resolve()),
        "out_root": str(out_root.resolve()),
        "prior_candidate_dir": str(prior_candidate_dir.resolve()),
        "stock_dataset_root": stock_summary["dataset_root"],
        "jmavsim_prior_dataset_root": prior_summary["dataset_root"],
        "jmavsim_prior_ident_root": str((ident_runs_root / "jmavsim_prior_ident_suite").resolve()),
        "reidentified_candidate_dir": str(reidentified_candidate_dir.resolve()),
        "reidentified_dataset_root": reidentified_summary["dataset_root"],
        "prior_prepare": prior_prepare,
        "reidentified_prepare": reidentified_prepare,
        "ident_manifest": str((ident_runs_root / "jmavsim_prior_ident_suite" / "run_manifest.json").resolve()),
        "candidate_summary": candidate_summary,
        "figures_summary": figures_summary,
    }
    summary_path = out_root / "three_model_validation_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    summary["summary_json"] = str(summary_path.resolve())
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Run the stock/prior/re-identified SITL validation matrix.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--out-root", required=True)
    ap.add_argument("--prior-candidate-dir", default="examples/paper_assets/candidates/jmavsim_prior_v1")
    ap.add_argument("--force", action="store_true", help="Re-run all stages even if cached outputs exist.")
    args = ap.parse_args()

    summary = run_three_model_validation(
        px4_root=Path(args.px4_root).expanduser().resolve(),
        out_root=Path(args.out_root).expanduser().resolve(),
        prior_candidate_dir=(REPO_ROOT / args.prior_candidate_dir).resolve()
        if not Path(args.prior_candidate_dir).is_absolute()
        else Path(args.prior_candidate_dir).expanduser().resolve(),
        force=args.force,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
