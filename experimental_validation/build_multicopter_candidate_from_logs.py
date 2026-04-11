#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import (
    TRUTH_POLICY_FULL,
    TRUTH_POLICY_NONE,
    TRUTH_POLICY_TELEMETRY,
    aggregate_identification_rows,
    build_identification_mode_reports,
    choose_primary_identification_mode,
    parse_multicopter_sdf_reference,
    resolve_truth_csvs,
    write_comparison_outputs,
)
from experimental_validation.identification import estimate_parameters_from_identification_log
from experimental_validation.sdf_export import build_inertial_snippet, build_parameter_summary

REQUIRED_PROFILES = (
    "hover_thrust",
    "mass_vertical",
    "roll_sweep",
    "pitch_sweep",
    "yaw_sweep",
    "drag_x",
    "drag_y",
    "drag_z",
    "motor_step",
)

OPTIONAL_PROFILES = (
    "actuator_lag_collective",
    "bridge_probe_xy",
)


def latest_profile_logs(ident_root: Path) -> tuple[dict[str, Path], list[str]]:
    selected: dict[str, Path] = {}
    missing: list[str] = []

    for profile in REQUIRED_PROFILES:
        matches = sorted(ident_root.glob(f"{profile}*.csv"), key=lambda path: path.stat().st_mtime)
        if not matches:
            missing.append(profile)
            continue
        selected[profile] = matches[-1].resolve()

    for profile in OPTIONAL_PROFILES:
        matches = sorted(ident_root.glob(f"{profile}*.csv"), key=lambda path: path.stat().st_mtime)
        if matches:
            selected[profile] = matches[-1].resolve()

    return selected, missing


def latest_truth_csvs(truth_root: Path) -> list[Path]:
    if not truth_root.exists():
        return []
    matches = sorted(truth_root.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    return [path.resolve() for path in matches]


def build_identification_reports_without_reference(
    csv_paths: list[Path],
    *,
    explicit_truth_csvs: list[Path] | None = None,
    disable_truth_autodetect: bool = False,
    prior_hints: dict | None = None,
    prefer_prior_mass_anchor: bool = False,
) -> dict[str, dict]:
    truth_csv_map = resolve_truth_csvs(
        csv_paths,
        explicit_truth_csvs,
        disable_autodetect=disable_truth_autodetect,
    )
    mode_specs = {
        "px4_only": TRUTH_POLICY_NONE,
        "telemetry_augmented": TRUTH_POLICY_TELEMETRY,
        "truth_assisted": TRUTH_POLICY_FULL,
    }
    reports: dict[str, dict] = {}
    for mode_name, truth_policy in mode_specs.items():
        rows, profile_counts, used_truth_csvs = aggregate_identification_rows(
            csv_paths,
            truth_policy=truth_policy,
            truth_csv_map=truth_csv_map,
        )
        reports[mode_name] = {
            "truth_policy": truth_policy,
            "effective_truth_policy": truth_policy if used_truth_csvs else TRUTH_POLICY_NONE,
            "truth_available": bool(used_truth_csvs),
            "truth_csvs": [str(path) for path in used_truth_csvs],
            "profile_counts": profile_counts,
            "identified": estimate_parameters_from_identification_log(
                rows,
                prior_hints=prior_hints,
                prefer_prior_mass_anchor=prefer_prior_mass_anchor,
            ),
        }
    return reports


def write_reference_free_outputs(
    out_dir: Path,
    *,
    reports_by_mode: dict[str, dict],
    csv_paths: list[Path],
    primary_mode: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    identified = reports_by_mode[primary_mode]["identified"]
    (out_dir / "identified_parameters.json").write_text(json.dumps(identified, indent=2), encoding="utf-8")
    (out_dir / "identified_parameters_by_mode.json").write_text(
        json.dumps({mode: payload["identified"] for mode, payload in reports_by_mode.items()}, indent=2),
        encoding="utf-8",
    )
    (out_dir / "used_identification_logs.json").write_text(
        json.dumps(
            {
                "logs": [str(path) for path in csv_paths],
                "primary_mode": primary_mode,
                "modes": {
                    mode: {
                        "truth_policy": payload["truth_policy"],
                        "effective_truth_policy": payload["effective_truth_policy"],
                        "truth_available": payload["truth_available"],
                        "truth_csvs": payload["truth_csvs"],
                        "profile_counts": payload["profile_counts"],
                    }
                    for mode, payload in reports_by_mode.items()
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    mass_kg = float(identified["mass"]["mass_kg"])
    ixx = float(identified["inertia"]["x"]["inertia_kgm2"])
    iyy = float(identified["inertia"]["y"]["inertia_kgm2"])
    izz = float(identified["inertia"]["z"]["inertia_kgm2"])
    (out_dir / "candidate_inertial.sdf.xml").write_text(
        build_inertial_snippet(mass_kg=mass_kg, ixx=ixx, iyy=iyy, izz=izz) + "\n",
        encoding="utf-8",
    )
    (out_dir / "candidate_vehicle_params.yaml").write_text(
        build_parameter_summary(
            mass_kg=mass_kg,
            thrust_scale_n_per_cmd=float(identified["thrust_scale"]["thrust_scale_n_per_cmd"]),
            drag_coeff_x=float(identified["drag"]["x"]["coefficient"]),
            drag_coeff_y=float(identified["drag"]["y"]["coefficient"]),
            drag_coeff_z=float(identified["drag"]["z"]["coefficient"]),
        ) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Build one multicopter candidate from an identification_logs directory.")
    ap.add_argument("--ident-root", required=True, help="Directory containing hover_thrust...motor_step identification CSVs.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--truth-root", default="", help="Optional directory containing truth CSVs; if given, the latest one is used.")
    ap.add_argument("--truth-csv", default="", help="Optional explicit truth CSV override.")
    ap.add_argument("--disable-truth-autodetect", action="store_true", help="Do not auto-pick sibling truth CSVs when no truth path is provided.")
    ap.add_argument("--mass-anchor-thrust-scale", type=float, default=None, help="Optional research-mode thrust-scale anchor (N per normalized command) used to convert collective specific-force gain into a mass estimate.")
    ap.add_argument("--mass-anchor-label", default="prior_thrust_scale_anchor", help="Label recorded for the optional mass anchor.")
    ap.add_argument("--prefer-prior-mass-anchor", action="store_true", help="If pure no-truth mass is unobservable, use the supplied thrust-scale anchor to set the reported mass instead of the nominal 1.0 kg fallback.")
    ap.add_argument("--sdf-model", default="", help="Optional reference multicopter model SDF path.")
    ap.add_argument("--sdf-base-model", default="", help="Optional base model SDF path. If omitted, the parser tries a sibling base model.")
    args = ap.parse_args()

    ident_root = Path(args.ident_root).expanduser().resolve()
    if not ident_root.exists():
        raise SystemExit(f"identification root not found: {ident_root}")

    logs_by_profile, missing = latest_profile_logs(ident_root)
    if missing:
        print(
            json.dumps(
                {
                    "ok": False,
                    "identification_logs": str(ident_root),
                    "missing_profiles": missing,
                },
                indent=2,
            )
        )
        return 2

    truth_csvs: list[Path] = []
    if args.truth_csv:
        truth_csvs = [Path(args.truth_csv).expanduser().resolve()]
    elif args.truth_root:
        truth_csvs = latest_truth_csvs(Path(args.truth_root).expanduser().resolve())

    csv_paths = [logs_by_profile[profile] for profile in REQUIRED_PROFILES]
    for profile in OPTIONAL_PROFILES:
        path = logs_by_profile.get(profile)
        if path is not None:
            csv_paths.append(path)
    out_dir = Path(args.out_dir).expanduser().resolve()
    prior_hints = None
    if args.mass_anchor_thrust_scale is not None:
        prior_hints = {
            "label": args.mass_anchor_label,
            "thrust_scale_n_per_cmd": float(args.mass_anchor_thrust_scale),
        }
    if args.sdf_model:
        sdf_reference = parse_multicopter_sdf_reference(args.sdf_model, args.sdf_base_model or None)
        reports_by_mode = build_identification_mode_reports(
            csv_paths,
            sdf_reference,
            explicit_truth_csvs=truth_csvs,
            disable_truth_autodetect=args.disable_truth_autodetect,
        )
        primary_mode = choose_primary_identification_mode(reports_by_mode)
        write_comparison_outputs(
            out_dir,
            reports_by_mode=reports_by_mode,
            sdf_reference=sdf_reference,
            csv_paths=csv_paths,
            primary_mode=primary_mode,
        )
        reference_model = str(sdf_reference.get("model_name", ""))
    else:
        reports_by_mode = build_identification_reports_without_reference(
            csv_paths,
            explicit_truth_csvs=truth_csvs,
            disable_truth_autodetect=args.disable_truth_autodetect,
            prior_hints=prior_hints,
            prefer_prior_mass_anchor=args.prefer_prior_mass_anchor,
        )
        primary_mode = choose_primary_identification_mode(reports_by_mode)
        write_reference_free_outputs(
            out_dir,
            reports_by_mode=reports_by_mode,
            csv_paths=csv_paths,
            primary_mode=primary_mode,
        )
        reference_model = ""

    print(
        json.dumps(
            {
                "ok": True,
                "ident_root": str(ident_root),
                "out_dir": str(out_dir),
                "reference_model": reference_model,
                "primary_mode": primary_mode,
                "truth_csvs": reports_by_mode[primary_mode]["truth_csvs"],
                "selected_logs": {profile: str(path) for profile, path in logs_by_profile.items()},
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
