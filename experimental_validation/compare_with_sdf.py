"""Compare identified Gazebo parameters against an SDF vehicle model."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.identification import (
    estimate_parameters_from_identification_log,
    load_identification_csv,
)
from experimental_validation.sdf_export import (
    apply_inertial_snippet_to_sdf,
    build_inertial_snippet,
    build_parameter_summary,
)
from experimental_validation.twin_metrics import build_blended_twin_score

TRUTH_POLICY_NONE = "none"
TRUTH_POLICY_TELEMETRY = "telemetry"
TRUTH_POLICY_FULL = "full"
PRIMARY_IDENTIFICATION_MODE = "truth_assisted"
MAX_TRUTH_SESSION_MTIME_GAP_S = 600.0
TELEMETRY_TRUTH_FIELDS = frozenset(
    {
        "sim_time_us",
        "observed_max_rot_velocity_radps",
        *{f"rotor_{i}_actual_radps" for i in range(4)},
        *{f"rotor_{i}_joint_vel_radps" for i in range(4)},
        *{f"rotor_{i}_cmd_radps" for i in range(4)},
    }
)
TELEMETRY_MERGED_MARKERS = frozenset(
    {
        "observed_max_rot_velocity_radps",
        *{f"rotor_{i}_actual_radps" for i in range(4)},
        *{f"rotor_{i}_joint_vel_radps" for i in range(4)},
    }
)
FULL_TRUTH_MARKERS = frozenset(
    {
        "truth_mass_kg",
        "truth_ixx_kgm2",
        "truth_iyy_kgm2",
        "truth_izz_kgm2",
        "truth_time_constant_up_s",
        "truth_time_constant_down_s",
        "truth_max_rot_velocity_radps",
        "truth_motor_constant",
        "truth_moment_constant",
        "truth_rotor_drag_coefficient",
        "truth_rolling_moment_coefficient",
        "truth_rotor_velocity_slowdown_sim",
        "thrust_n",
        "total_prop_thrust_n",
        *{f"rotor_{i}_actual_radps" for i in range(4)},
    }
)


def _find_first_text(root: ET.Element, path: str, default: float = 0.0) -> float:
    element = root.find(path)
    if element is None or element.text is None:
        return float(default)
    return float(element.text.strip())


def resolve_px4_reference_path(raw_path: str | Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path.resolve()

    candidates = [
        (REPO_ROOT / path).resolve(),
    ]
    for env_name in ("PX4_WORKSPACE", "PX4_SYSID_PX4_ROOT"):
        env_value = os.environ.get(env_name, "").strip()
        if env_value:
            candidates.append((Path(env_value).expanduser().resolve() / path).resolve())
    for sibling_name in ("px4-custom", "PX4-Autopilot-Identification", "PX4-Autopilot"):
        candidates.append((REPO_ROOT.parent / sibling_name / path).resolve())

    for candidate in candidates:
        if candidate.exists():
            return candidate

    searched = "\n".join(f"- {candidate}" for candidate in candidates)
    raise FileNotFoundError(f"Could not resolve PX4 reference path for '{raw_path}'. Searched:\n{searched}")


def parse_x500_sdf_reference(model_sdf: str | Path, base_model_sdf: str | Path | None = None) -> dict:
    model_path = resolve_px4_reference_path(model_sdf)
    if base_model_sdf:
        base_path = resolve_px4_reference_path(base_model_sdf)
    else:
        base_path = model_path.parent.parent / "x500_base" / "model.sdf"

    model_tree = ET.parse(model_path)
    model_root = model_tree.getroot()
    base_tree = ET.parse(base_path)
    base_root = base_tree.getroot()

    base_link = base_root.find(".//link[@name='base_link']")
    if base_link is None:
        raise RuntimeError(f"base_link missing in SDF: {base_path}")

    motor_plugins = base_root.findall(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    if not motor_plugins:
        motor_plugins = model_root.findall(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']")

    def plugin_avg(tag: str, default: float = 0.0) -> float:
        values: list[float] = []
        for plugin in motor_plugins:
            element = plugin.find(tag)
            if element is not None and element.text is not None:
                values.append(float(element.text.strip()))
        if not values:
            return float(default)
        return float(sum(values) / len(values))

    return {
        "model_name": model_root.find(".//model").attrib.get("name", model_path.stem),
        "model_sdf": str(model_path),
        "base_model_sdf": str(base_path),
        "mass": {
            "mass_kg": _find_first_text(base_link, "./inertial/mass"),
        },
        "inertia": {
            "x": {"inertia_kgm2": _find_first_text(base_link, "./inertial/inertia/ixx")},
            "y": {"inertia_kgm2": _find_first_text(base_link, "./inertial/inertia/iyy")},
            "z": {"inertia_kgm2": _find_first_text(base_link, "./inertial/inertia/izz")},
        },
        "motor_model": {
            "time_constant_up_s": plugin_avg("timeConstantUp"),
            "time_constant_down_s": plugin_avg("timeConstantDown"),
            "max_rot_velocity_radps": plugin_avg("maxRotVelocity"),
            "motor_constant": plugin_avg("motorConstant"),
            "moment_constant": plugin_avg("momentConstant"),
            "rotor_drag_coefficient": plugin_avg("rotorDragCoefficient"),
            "rolling_moment_coefficient": plugin_avg("rollingMomentCoefficient"),
            "rotor_velocity_slowdown_sim": plugin_avg("rotorVelocitySlowdownSim"),
        },
    }


def collect_identification_logs(results_root: str | Path, *, latest_only: bool = True) -> list[Path]:
    root = Path(results_root).resolve()
    if not root.exists():
        raise RuntimeError(f"results root not found: {root}")

    if latest_only:
        logs: list[Path] = []
        for trace_dir in sorted(root.glob("*/identification_traces")):
            evals = sorted(trace_dir.glob("eval_*.csv"))
            if evals:
                logs.append(evals[-1])
    else:
        logs = sorted(root.glob("*/identification_traces/eval_*.csv"))

    if not logs:
        raise RuntimeError(f"no identification traces found under: {root}")
    return logs


def _autodetect_truth_csv_for_ident_log(path: Path) -> Path | None:
    exact_candidates = [
        path.parents[1] / "gazebo_truth_traces" / path.name,
    ]
    for candidate in exact_candidates:
        if candidate.exists():
            return candidate.resolve()

    directory_candidates = [
        path.parents[1] / "sysid_truth_logs",
        path.parents[1] / "gazebo_truth_traces",
    ]
    for directory in directory_candidates:
        if not directory.exists():
            continue
        csvs = sorted(directory.glob("*.csv"), key=lambda item: item.stat().st_mtime)
        if not csvs:
            continue
        if directory.name == "sysid_truth_logs":
            ident_mtime = path.stat().st_mtime
            scored = sorted(csvs, key=lambda item: abs(item.stat().st_mtime - ident_mtime))
            nearest = scored[0]
            if abs(nearest.stat().st_mtime - ident_mtime) <= MAX_TRUTH_SESSION_MTIME_GAP_S:
                return nearest.resolve()
            continue
        return csvs[-1].resolve()
    return None


def resolve_truth_csvs(
    csv_paths: Iterable[str | Path],
    explicit_truth_csvs: Iterable[str | Path] | None = None,
) -> dict[Path, Path | None]:
    csv_paths = [Path(path).resolve() for path in csv_paths]
    explicit = [Path(path).resolve() for path in (explicit_truth_csvs or [])]

    mapping: dict[Path, Path | None] = {}
    if len(explicit) == 1:
        for csv_path in csv_paths:
            mapping[csv_path] = explicit[0]
        return mapping

    if len(explicit) == len(csv_paths) and explicit:
        for csv_path, truth_path in zip(csv_paths, explicit):
            mapping[csv_path] = truth_path
        return mapping

    if explicit:
        raise RuntimeError(
            "Provide either one --truth-csv for the whole session or one --truth-csv per --csv."
        )

    for csv_path in csv_paths:
        mapping[csv_path] = _autodetect_truth_csv_for_ident_log(csv_path)
    return mapping


def _rows_have_truth_fields(rows: list[dict[str, float | str]], *, truth_policy: str) -> bool:
    marker_keys = set(TELEMETRY_MERGED_MARKERS)
    if truth_policy == TRUTH_POLICY_FULL:
        marker_keys |= set(FULL_TRUTH_MARKERS)
    for row in rows:
        if any(key in row for key in marker_keys):
            return True
    return False


def aggregate_identification_rows(
    csv_paths: Iterable[str | Path],
    *,
    truth_policy: str = TRUTH_POLICY_TELEMETRY,
    truth_csv_map: dict[Path, Path | None] | None = None,
) -> tuple[list[dict[str, float | str]], dict[str, int], list[Path]]:
    if truth_policy not in {TRUTH_POLICY_NONE, TRUTH_POLICY_TELEMETRY, TRUTH_POLICY_FULL}:
        raise ValueError(f"unsupported truth_policy: {truth_policy}")
    rows: list[dict[str, float | str]] = []
    counts: dict[str, int] = {}
    used_truth_csvs: list[Path] = []
    for run_index, path in enumerate(csv_paths):
        path = Path(path)
        truth_path = truth_csv_map.get(path.resolve()) if truth_csv_map else _autodetect_truth_csv_for_ident_log(path)
        allow_relative_alignment = truth_path is not None and Path(truth_path).parent.name != "sysid_truth_logs"
        if truth_policy == TRUTH_POLICY_NONE or truth_path is None or not truth_path.exists():
            csv_rows = load_identification_csv(path)
        elif truth_policy == TRUTH_POLICY_TELEMETRY:
            csv_rows = load_identification_csv(
                path,
                truth_csv=truth_path,
                truth_field_allowlist=set(TELEMETRY_TRUTH_FIELDS),
                allow_relative_truth_alignment=allow_relative_alignment,
            )
            if _rows_have_truth_fields(csv_rows, truth_policy=truth_policy):
                used_truth_csvs.append(Path(truth_path).resolve())
        else:
            csv_rows = load_identification_csv(
                path,
                truth_csv=truth_path,
                allow_relative_truth_alignment=allow_relative_alignment,
            )
            if _rows_have_truth_fields(csv_rows, truth_policy=truth_policy):
                used_truth_csvs.append(Path(truth_path).resolve())
        for row in csv_rows:
            row["run_index"] = float(run_index)
        rows.extend(csv_rows)
        for row in csv_rows:
            profile = str(row.get("profile") or "unknown")
            counts[profile] = counts.get(profile, 0) + 1
    if not rows:
        raise RuntimeError("no identification rows loaded")
    unique_truths = sorted(set(used_truth_csvs))
    return rows, counts, unique_truths


def build_identification_mode_reports(
    csv_paths: Iterable[str | Path],
    sdf_reference: dict,
    *,
    explicit_truth_csvs: Iterable[str | Path] | None = None,
) -> dict:
    csv_paths = [Path(path).resolve() for path in csv_paths]
    truth_csv_map = resolve_truth_csvs(csv_paths, explicit_truth_csvs)
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
        identified = estimate_parameters_from_identification_log(rows)
        comparison = compare_identified_to_sdf(identified, sdf_reference)
        reports[mode_name] = {
            "truth_policy": truth_policy,
            "effective_truth_policy": truth_policy if used_truth_csvs else TRUTH_POLICY_NONE,
            "truth_available": bool(used_truth_csvs),
            "truth_csvs": [str(path) for path in used_truth_csvs],
            "profile_counts": profile_counts,
            "identified": identified,
            "comparison": comparison,
        }
    return reports


def choose_primary_identification_mode(reports_by_mode: dict[str, dict]) -> str:
    if reports_by_mode.get("truth_assisted", {}).get("truth_available"):
        return "truth_assisted"
    if reports_by_mode.get("telemetry_augmented", {}).get("truth_available"):
        return "telemetry_augmented"
    return "px4_only"


def _metric_error(identified_value: float, reference_value: float) -> dict:
    abs_err = float(identified_value) - float(reference_value)
    pct_err = 0.0 if abs(reference_value) < 1e-9 else (abs_err / float(reference_value)) * 100.0
    return {
        "identified": float(identified_value),
        "reference": float(reference_value),
        "abs_error": abs_err,
        "pct_error": pct_err,
    }


def compare_identified_to_sdf(identified: dict, sdf_reference: dict) -> dict:
    comparable = {
        "mass_kg": _metric_error(
            float(identified["mass"]["mass_kg"]),
            float(sdf_reference["mass"]["mass_kg"]),
        ),
        "ixx_kgm2": _metric_error(
            float(identified["inertia"]["x"]["inertia_kgm2"]),
            float(sdf_reference["inertia"]["x"]["inertia_kgm2"]),
        ),
        "iyy_kgm2": _metric_error(
            float(identified["inertia"]["y"]["inertia_kgm2"]),
            float(sdf_reference["inertia"]["y"]["inertia_kgm2"]),
        ),
        "izz_kgm2": _metric_error(
            float(identified["inertia"]["z"]["inertia_kgm2"]),
            float(sdf_reference["inertia"]["z"]["inertia_kgm2"]),
        ),
        "time_constant_up_s": _metric_error(
            float(identified["motor_model"]["time_constant_up_s"]["value"]),
            float(sdf_reference["motor_model"]["time_constant_up_s"]),
        ),
        "time_constant_down_s": _metric_error(
            float(identified["motor_model"]["time_constant_down_s"]["value"]),
            float(sdf_reference["motor_model"]["time_constant_down_s"]),
        ),
        "max_rot_velocity_radps": _metric_error(
            float(identified["motor_model"]["max_rot_velocity_radps"]["value"]),
            float(sdf_reference["motor_model"]["max_rot_velocity_radps"]),
        ),
        "motor_constant": _metric_error(
            float(identified["motor_model"]["motor_constant"]["value"]),
            float(sdf_reference["motor_model"]["motor_constant"]),
        ),
        "moment_constant": _metric_error(
            float(identified["motor_model"]["moment_constant"]["value"]),
            float(sdf_reference["motor_model"]["moment_constant"]),
        ),
        "rotor_drag_coefficient": _metric_error(
            float(identified["motor_model"]["rotor_drag_coefficient"]["value"]),
            float(sdf_reference["motor_model"]["rotor_drag_coefficient"]),
        ),
        "rolling_moment_coefficient": _metric_error(
            float(identified["motor_model"]["rolling_moment_coefficient"]["value"]),
            float(sdf_reference["motor_model"]["rolling_moment_coefficient"]),
        ),
        "rotor_velocity_slowdown_sim": _metric_error(
            float(identified["motor_model"]["rotor_velocity_slowdown_sim"]["value"]),
            float(sdf_reference["motor_model"]["rotor_velocity_slowdown_sim"]),
        ),
    }
    blended_twin_score = build_blended_twin_score(comparable)
    return {
        "comparable_metrics": comparable,
        "blended_twin_score": blended_twin_score,
        "non_comparable_metrics": {
            "identified_only": [
                "thrust_scale_n_per_cmd",
                "drag_x",
                "drag_y",
                "drag_z",
            ],
            "sdf_only": [],
            "note": (
                "Mass, inertia, and Gazebo motor-model terms are compared directly. "
                "Generic body drag and thrust-scale terms remain identification-only summaries."
            ),
        },
    }


def write_comparison_outputs(
    out_dir: Path,
    *,
    reports_by_mode: dict[str, dict],
    sdf_reference: dict,
    csv_paths: list[Path],
    primary_mode: str = PRIMARY_IDENTIFICATION_MODE,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    primary_report = reports_by_mode[primary_mode]
    identified = primary_report["identified"]
    comparison = primary_report["comparison"]
    (out_dir / "identified_parameters.json").write_text(json.dumps(identified, indent=2), encoding="utf-8")
    (out_dir / "sdf_reference.json").write_text(json.dumps(sdf_reference, indent=2), encoding="utf-8")
    (out_dir / "sdf_comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    (out_dir / "identified_parameters_by_mode.json").write_text(
        json.dumps({mode: payload["identified"] for mode, payload in reports_by_mode.items()}, indent=2),
        encoding="utf-8",
    )
    (out_dir / "sdf_comparison_by_mode.json").write_text(
        json.dumps({mode: payload["comparison"] for mode, payload in reports_by_mode.items()}, indent=2),
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
        )
        + "\n",
        encoding="utf-8",
    )
    base_model_path = Path(str(sdf_reference["base_model_sdf"]))
    base_model_text = base_model_path.read_text(encoding="utf-8")
    patched = apply_inertial_snippet_to_sdf(
        base_model_text,
        build_inertial_snippet(mass_kg=mass_kg, ixx=ixx, iyy=iyy, izz=izz),
    )
    (out_dir / "candidate_x500_base.sdf").write_text(patched, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare identified Gazebo parameters against an SDF reference.")
    ap.add_argument(
        "--results-root",
        action="append",
        default=[],
        help="Root directory that contains per-sortie subfolders with identification_traces. Use multiple times to combine sorties.",
    )
    ap.add_argument("--csv", action="append", default=[], help="Identification CSV file(s). Use multiple times.")
    ap.add_argument("--out-dir", required=True, help="Output directory for reports.")
    ap.add_argument("--truth-csv", action="append", default=[], help="Optional Gazebo truth CSV. Provide one for the whole session or one per --csv.")
    ap.add_argument("--sdf-model", default="Tools/simulation/gz/models/x500/model.sdf")
    ap.add_argument("--sdf-base-model", default="Tools/simulation/gz/models/x500_base/model.sdf")
    ap.add_argument("--all-evals", action="store_true", help="Use every eval_*.csv under each identification task instead of only the latest eval.")
    args = ap.parse_args()

    repo_root = REPO_ROOT
    csv_paths: list[Path] = []
    for raw_root in args.results_root:
        root = Path(raw_root)
        if not root.is_absolute():
            root = (repo_root / root).resolve()
        csv_paths.extend(collect_identification_logs(root, latest_only=not args.all_evals))
    for raw in args.csv:
        path = Path(raw)
        csv_paths.append((repo_root / path).resolve() if not path.is_absolute() else path.resolve())
    if not csv_paths:
        raise RuntimeError("provide --results-root or at least one --csv")

    sdf_reference = parse_x500_sdf_reference(
        Path(args.sdf_model).resolve() if Path(args.sdf_model).is_absolute() else args.sdf_model,
        Path(args.sdf_base_model).resolve() if Path(args.sdf_base_model).is_absolute() else args.sdf_base_model,
    )
    reports_by_mode = build_identification_mode_reports(csv_paths, sdf_reference, explicit_truth_csvs=args.truth_csv)
    primary_mode = choose_primary_identification_mode(reports_by_mode)
    out_dir = (repo_root / args.out_dir).resolve() if not Path(args.out_dir).is_absolute() else Path(args.out_dir).resolve()
    write_comparison_outputs(
        out_dir,
        reports_by_mode=reports_by_mode,
        sdf_reference=sdf_reference,
        csv_paths=csv_paths,
        primary_mode=primary_mode,
    )
    primary_report = reports_by_mode[primary_mode]
    print(json.dumps({
        "ok": True,
        "out_dir": str(out_dir),
        "logs": [str(path) for path in csv_paths],
        "primary_mode": primary_mode,
        "truth_csvs": primary_report["truth_csvs"],
        "profile_counts": primary_report["profile_counts"],
        "comparison": primary_report["comparison"]["comparable_metrics"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
