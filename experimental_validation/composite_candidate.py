"""Compose a digital-twin candidate from trusted identification source families."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import compare_identified_to_sdf, parse_x500_sdf_reference
from experimental_validation.sdf_export import (
    apply_inertial_snippet_to_sdf,
    build_inertial_snippet,
    build_parameter_summary,
)


def load_identified_output(path: str | Path) -> dict:
    root = Path(path).resolve()
    file_path = root / "identified_parameters.json" if root.is_dir() else root
    if not file_path.exists():
        raise FileNotFoundError(f"identified output not found: {file_path}")
    return json.loads(file_path.read_text(encoding="utf-8"))


def build_composite_identified(
    *,
    mass_source: dict,
    inertia_source: dict,
    drag_source: dict,
    motor_source: dict,
) -> dict:
    warnings: list[str] = []
    for label, source in (
        ("mass_source", mass_source),
        ("inertia_source", inertia_source),
        ("drag_source", drag_source),
        ("motor_source", motor_source),
    ):
        for warning in source.get("warnings", []):
            warnings.append(f"{label}: {warning}")

    composite = {
        "mass": mass_source["mass"],
        "thrust_scale": mass_source["thrust_scale"],
        "inertia": inertia_source["inertia"],
        "drag": drag_source["drag"],
        "motor_model": motor_source["motor_model"],
        "warnings": warnings,
        "composite_sources": {
            "mass": mass_source.get("source_name", "mass_source"),
            "inertia": inertia_source.get("source_name", "inertia_source"),
            "drag": drag_source.get("source_name", "drag_source"),
            "motor_model": motor_source.get("source_name", "motor_source"),
        },
    }
    return composite


def write_composite_outputs(
    out_dir: str | Path,
    *,
    identified: dict,
    sdf_reference: dict,
    source_paths: dict[str, str],
) -> None:
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    comparison = compare_identified_to_sdf(identified, sdf_reference)
    (out_dir / "identified_parameters.json").write_text(json.dumps(identified, indent=2), encoding="utf-8")
    (out_dir / "sdf_reference.json").write_text(json.dumps(sdf_reference, indent=2), encoding="utf-8")
    (out_dir / "sdf_comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    (out_dir / "composite_sources.json").write_text(json.dumps(source_paths, indent=2), encoding="utf-8")

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
    ap = argparse.ArgumentParser(description="Compose a digital-twin candidate from multiple identification outputs.")
    ap.add_argument("--mass-source", required=True, help="Output directory containing identified_parameters.json for mass/thrust.")
    ap.add_argument("--inertia-source", required=True, help="Output directory containing identified_parameters.json for inertia.")
    ap.add_argument("--drag-source", required=True, help="Output directory containing identified_parameters.json for drag.")
    ap.add_argument("--motor-source", required=True, help="Output directory containing identified_parameters.json for motor terms.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--sdf-model", default="Tools/simulation/gz/models/x500/model.sdf")
    ap.add_argument("--sdf-base-model", default="Tools/simulation/gz/models/x500_base/model.sdf")
    args = ap.parse_args()

    source_paths = {
        "mass": str(Path(args.mass_source).resolve()),
        "inertia": str(Path(args.inertia_source).resolve()),
        "drag": str(Path(args.drag_source).resolve()),
        "motor_model": str(Path(args.motor_source).resolve()),
    }
    mass_source = load_identified_output(args.mass_source)
    mass_source["source_name"] = source_paths["mass"]
    inertia_source = load_identified_output(args.inertia_source)
    inertia_source["source_name"] = source_paths["inertia"]
    drag_source = load_identified_output(args.drag_source)
    drag_source["source_name"] = source_paths["drag"]
    motor_source = load_identified_output(args.motor_source)
    motor_source["source_name"] = source_paths["motor_model"]

    identified = build_composite_identified(
        mass_source=mass_source,
        inertia_source=inertia_source,
        drag_source=drag_source,
        motor_source=motor_source,
    )
    sdf_reference = parse_x500_sdf_reference(Path(args.sdf_model), Path(args.sdf_base_model))
    write_composite_outputs(args.out_dir, identified=identified, sdf_reference=sdf_reference, source_paths=source_paths)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
