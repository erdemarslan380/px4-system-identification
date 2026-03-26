"""Compare identified Gazebo parameters against an SDF vehicle model."""

from __future__ import annotations

import argparse
import json
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

TRUTH_POLICY_NONE = "none"
TRUTH_POLICY_TELEMETRY = "telemetry"
TRUTH_POLICY_FULL = "full"
PRIMARY_IDENTIFICATION_MODE = "truth_assisted"
TELEMETRY_TRUTH_FIELDS = frozenset(
    {
        "sim_time_us",
        "observed_max_rot_velocity_radps",
        *{f"rotor_{i}_actual_radps" for i in range(4)},
        *{f"rotor_{i}_joint_vel_radps" for i in range(4)},
        *{f"rotor_{i}_cmd_radps" for i in range(4)},
    }
)


def _find_first_text(root: ET.Element, path: str, default: float = 0.0) -> float:
    element = root.find(path)
    if element is None or element.text is None:
        return float(default)
    return float(element.text.strip())


def parse_x500_sdf_reference(model_sdf: str | Path, base_model_sdf: str | Path | None = None) -> dict:
    model_path = Path(model_sdf).resolve()
    if base_model_sdf:
        base_path = Path(base_model_sdf).resolve()
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


def aggregate_identification_rows(
    csv_paths: Iterable[str | Path],
    *,
    truth_policy: str = TRUTH_POLICY_TELEMETRY,
) -> tuple[list[dict[str, float | str]], dict[str, int]]:
    if truth_policy not in {TRUTH_POLICY_NONE, TRUTH_POLICY_TELEMETRY, TRUTH_POLICY_FULL}:
        raise ValueError(f"unsupported truth_policy: {truth_policy}")
    rows: list[dict[str, float | str]] = []
    counts: dict[str, int] = {}
    for run_index, path in enumerate(csv_paths):
        path = Path(path)
        truth_path = path.parents[1] / "gazebo_truth_traces" / path.name
        if truth_policy == TRUTH_POLICY_NONE or not truth_path.exists():
            csv_rows = load_identification_csv(path)
        elif truth_policy == TRUTH_POLICY_TELEMETRY:
            csv_rows = load_identification_csv(path, truth_csv=truth_path, truth_field_allowlist=set(TELEMETRY_TRUTH_FIELDS))
        else:
            csv_rows = load_identification_csv(path, truth_csv=truth_path)
        for row in csv_rows:
            row["run_index"] = float(run_index)
        rows.extend(csv_rows)
        for row in csv_rows:
            profile = str(row.get("profile") or "unknown")
            counts[profile] = counts.get(profile, 0) + 1
    if not rows:
        raise RuntimeError("no identification rows loaded")
    return rows, counts


def build_identification_mode_reports(csv_paths: Iterable[str | Path], sdf_reference: dict) -> dict:
    csv_paths = [Path(path) for path in csv_paths]
    mode_specs = {
        "px4_only": TRUTH_POLICY_NONE,
        "telemetry_augmented": TRUTH_POLICY_TELEMETRY,
        "truth_assisted": TRUTH_POLICY_FULL,
    }
    reports: dict[str, dict] = {}
    for mode_name, truth_policy in mode_specs.items():
        rows, profile_counts = aggregate_identification_rows(csv_paths, truth_policy=truth_policy)
        identified = estimate_parameters_from_identification_log(rows)
        comparison = compare_identified_to_sdf(identified, sdf_reference)
        reports[mode_name] = {
            "truth_policy": truth_policy,
            "profile_counts": profile_counts,
            "identified": identified,
            "comparison": comparison,
        }
    return reports


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
    return {
        "comparable_metrics": comparable,
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
        (repo_root / args.sdf_model).resolve() if not Path(args.sdf_model).is_absolute() else Path(args.sdf_model).resolve(),
        (repo_root / args.sdf_base_model).resolve() if not Path(args.sdf_base_model).is_absolute() else Path(args.sdf_base_model).resolve(),
    )
    reports_by_mode = build_identification_mode_reports(csv_paths, sdf_reference)
    out_dir = (repo_root / args.out_dir).resolve() if not Path(args.out_dir).is_absolute() else Path(args.out_dir).resolve()
    write_comparison_outputs(
        out_dir,
        reports_by_mode=reports_by_mode,
        sdf_reference=sdf_reference,
        csv_paths=csv_paths,
    )
    primary_report = reports_by_mode[PRIMARY_IDENTIFICATION_MODE]
    print(json.dumps({
        "ok": True,
        "out_dir": str(out_dir),
        "logs": [str(path) for path in csv_paths],
        "primary_mode": PRIMARY_IDENTIFICATION_MODE,
        "profile_counts": primary_report["profile_counts"],
        "comparison": primary_report["comparison"]["comparable_metrics"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
