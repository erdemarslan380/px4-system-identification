#!/usr/bin/env python3
"""Estimate Gazebo SDF parameters from exported real-flight CSV data."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.estimators import (
    estimate_axis_inertia,
    estimate_hover_mass,
    estimate_quadratic_drag,
    estimate_thrust_scale,
    load_numeric_csv,
)
from experimental_validation.identification import estimate_parameters_from_identification_log, load_identification_csv
from experimental_validation.qgc_params import parse_qgc_parameter_dump
from experimental_validation.sdf_export import (
    apply_inertial_snippet_to_sdf,
    build_inertial_snippet,
    build_parameter_summary,
)


def main() -> int:
    ap = argparse.ArgumentParser(description="Estimate Gazebo SDF parameters from a CSV experiment log.")
    ap.add_argument("--csv", required=True, help="Input CSV file with experiment samples.")
    ap.add_argument("--truth-csv", default="", help="Optional Gazebo truth CSV paired with --csv.")
    ap.add_argument("--out-dir", required=True, help="Output directory for JSON and text reports.")
    ap.add_argument("--ident-log", action="store_true", help="Treat --csv as a PX4 identification log exported by trajectory_reader.")
    ap.add_argument("--qgc-param-file", default="", help="Optional QGroundControl full-parameter dump to bundle with outputs.")
    ap.add_argument("--sdf-template", default="", help="Optional SDF template file to patch with the identified inertial block.")
    ap.add_argument("--mass-thrust-column", default="thrust_n")
    ap.add_argument("--thrust-command-column", default="thrust_cmd")
    ap.add_argument("--accel-z-column", default="az_world_mps2")
    ap.add_argument("--roll-torque-column", default="tau_x_nm")
    ap.add_argument("--pitch-torque-column", default="tau_y_nm")
    ap.add_argument("--yaw-torque-column", default="tau_z_nm")
    ap.add_argument("--roll-accel-column", default="p_dot_radps2")
    ap.add_argument("--pitch-accel-column", default="q_dot_radps2")
    ap.add_argument("--yaw-accel-column", default="r_dot_radps2")
    ap.add_argument("--drag-x-velocity-column", default="vx_mps")
    ap.add_argument("--drag-x-accel-column", default="ax_drag_mps2")
    ap.add_argument("--drag-y-velocity-column", default="vy_mps")
    ap.add_argument("--drag-y-accel-column", default="ay_drag_mps2")
    ap.add_argument("--drag-z-velocity-column", default="vz_mps")
    ap.add_argument("--drag-z-accel-column", default="az_drag_mps2")
    args = ap.parse_args()

    if args.ident_log:
        rows = load_identification_csv(args.csv, truth_csv=args.truth_csv or None)
        base_report = estimate_parameters_from_identification_log(rows)
        mass = base_report["mass"]
        thrust = base_report["thrust_scale"]
        inertia_x = base_report["inertia"]["x"]
        inertia_y = base_report["inertia"]["y"]
        inertia_z = base_report["inertia"]["z"]
        drag_x = base_report["drag"]["x"]
        drag_y = base_report["drag"]["y"]
        drag_z = base_report["drag"]["z"]
    else:
        rows = load_numeric_csv(args.csv)
        mass = estimate_hover_mass(rows, thrust_column=args.mass_thrust_column, accel_z_column=args.accel_z_column)
        thrust = estimate_thrust_scale(
            rows,
            mass_kg=mass.mass_kg,
            command_column=args.thrust_command_column,
            accel_z_column=args.accel_z_column,
        )
        inertia_x = estimate_axis_inertia(rows, axis="x", torque_column=args.roll_torque_column, angular_accel_column=args.roll_accel_column)
        inertia_y = estimate_axis_inertia(rows, axis="y", torque_column=args.pitch_torque_column, angular_accel_column=args.pitch_accel_column)
        inertia_z = estimate_axis_inertia(rows, axis="z", torque_column=args.yaw_torque_column, angular_accel_column=args.yaw_accel_column)
        drag_x = estimate_quadratic_drag(rows, axis="x", mass_kg=mass.mass_kg, velocity_column=args.drag_x_velocity_column, accel_column=args.drag_x_accel_column)
        drag_y = estimate_quadratic_drag(rows, axis="y", mass_kg=mass.mass_kg, velocity_column=args.drag_y_velocity_column, accel_column=args.drag_y_accel_column)
        drag_z = estimate_quadratic_drag(rows, axis="z", mass_kg=mass.mass_kg, velocity_column=args.drag_z_velocity_column, accel_column=args.drag_z_accel_column)

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    report = {
        "mass": mass if isinstance(mass, dict) else mass.as_dict(),
        "thrust_scale": thrust if isinstance(thrust, dict) else thrust.as_dict(),
        "inertia": {
            "x": inertia_x if isinstance(inertia_x, dict) else inertia_x.as_dict(),
            "y": inertia_y if isinstance(inertia_y, dict) else inertia_y.as_dict(),
            "z": inertia_z if isinstance(inertia_z, dict) else inertia_z.as_dict(),
        },
        "drag": {
            "x": drag_x if isinstance(drag_x, dict) else drag_x.as_dict(),
            "y": drag_y if isinstance(drag_y, dict) else drag_y.as_dict(),
            "z": drag_z if isinstance(drag_z, dict) else drag_z.as_dict(),
        },
    }
    if args.ident_log and base_report.get("warnings"):
        report["warnings"] = list(base_report["warnings"])
    if args.ident_log and base_report.get("motor_model"):
        report["motor_model"] = dict(base_report["motor_model"])
    (out_dir / "identified_parameters.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    mass_kg = report["mass"]["mass_kg"]
    ixx = report["inertia"]["x"]["inertia_kgm2"]
    iyy = report["inertia"]["y"]["inertia_kgm2"]
    izz = report["inertia"]["z"]["inertia_kgm2"]
    thrust_scale_n_per_cmd = report["thrust_scale"]["thrust_scale_n_per_cmd"]
    (out_dir / "candidate_inertial.sdf.xml").write_text(
        build_inertial_snippet(
            mass_kg=mass_kg,
            ixx=ixx,
            iyy=iyy,
            izz=izz,
        )
        + "\n",
        encoding="utf-8",
    )
    (out_dir / "candidate_vehicle_params.yaml").write_text(
        build_parameter_summary(
            mass_kg=mass_kg,
            thrust_scale_n_per_cmd=thrust_scale_n_per_cmd,
            drag_coeff_x=report["drag"]["x"]["coefficient"],
            drag_coeff_y=report["drag"]["y"]["coefficient"],
            drag_coeff_z=report["drag"]["z"]["coefficient"],
        )
        + "\n",
        encoding="utf-8",
    )
    if args.qgc_param_file:
        params = parse_qgc_parameter_dump(args.qgc_param_file)
        (out_dir / "qgc_baseline_params.json").write_text(json.dumps(params, indent=2), encoding="utf-8")
    if args.sdf_template:
        template = Path(args.sdf_template).read_text(encoding="utf-8")
        patched = apply_inertial_snippet_to_sdf(
            template,
            build_inertial_snippet(mass_kg=mass_kg, ixx=ixx, iyy=iyy, izz=izz),
        )
        (out_dir / "candidate_model.sdf").write_text(patched, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
