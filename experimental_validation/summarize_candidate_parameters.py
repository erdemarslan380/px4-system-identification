from __future__ import annotations

import argparse
import json
from pathlib import Path


PARAM_ROWS = (
    ("Mass", "mass_kg", "kg", lambda d: float(d["mass"]["mass_kg"])),
    ("Thrust scale", "thrust_scale_n_per_cmd", "N/cmd", lambda d: float(d["thrust_scale"]["thrust_scale_n_per_cmd"])),
    ("Inertia X", "ixx_kgm2", "kg*m^2", lambda d: float(d["inertia"]["x"]["inertia_kgm2"])),
    ("Inertia Y", "iyy_kgm2", "kg*m^2", lambda d: float(d["inertia"]["y"]["inertia_kgm2"])),
    ("Inertia Z", "izz_kgm2", "kg*m^2", lambda d: float(d["inertia"]["z"]["inertia_kgm2"])),
    ("Drag X", "drag_x", "-", lambda d: float(d["drag"]["x"]["coefficient"])),
    ("Drag Y", "drag_y", "-", lambda d: float(d["drag"]["y"]["coefficient"])),
    ("Drag Z", "drag_z", "-", lambda d: float(d["drag"]["z"]["coefficient"])),
    ("Time constant up", "time_constant_up_s", "s", lambda d: float(d["motor_model"]["time_constant_up_s"]["value"])),
    ("Time constant down", "time_constant_down_s", "s", lambda d: float(d["motor_model"]["time_constant_down_s"]["value"])),
    ("Max rotor velocity", "max_rot_velocity_radps", "rad/s", lambda d: float(d["motor_model"]["max_rot_velocity_radps"]["value"])),
    ("Motor constant", "motor_constant", "N/(rad/s)^2", lambda d: float(d["motor_model"]["motor_constant"]["value"])),
    ("Moment constant", "moment_constant", "-", lambda d: float(d["motor_model"]["moment_constant"]["value"])),
    ("Rotor drag coeff", "rotor_drag_coefficient", "-", lambda d: float(d["motor_model"]["rotor_drag_coefficient"]["value"])),
    ("Rolling moment coeff", "rolling_moment_coefficient", "-", lambda d: float(d["motor_model"]["rolling_moment_coefficient"]["value"])),
    ("Velocity slowdown", "rotor_velocity_slowdown_sim", "-", lambda d: float(d["motor_model"]["rotor_velocity_slowdown_sim"]["value"])),
)


def _resolve_candidate_json(candidate: Path) -> Path:
    if candidate.is_dir():
        candidate = candidate / "identified_parameters.json"
    if not candidate.exists():
        raise FileNotFoundError(candidate)
    return candidate


def _load_candidate(candidate: Path) -> dict:
    path = _resolve_candidate_json(candidate)
    return json.loads(path.read_text(encoding="utf-8"))


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0"
    if abs(value) >= 1000.0 or abs(value) < 1e-3:
        return f"{value:.6e}"
    return f"{value:.6f}".rstrip("0").rstrip(".")


def _build_rows(candidate_a: dict, candidate_b: dict | None) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for label, key, unit, getter in PARAM_ROWS:
        a_value = getter(candidate_a)
        row = {
            "label": label,
            "key": key,
            "unit": unit,
            "value_a": a_value,
        }
        if candidate_b is not None:
            b_value = getter(candidate_b)
            row["value_b"] = b_value
            row["delta_abs"] = b_value - a_value
            row["delta_pct"] = ((b_value - a_value) / a_value * 100.0) if a_value != 0.0 else None
        rows.append(row)
    return rows


def _build_markdown(label_a: str, rows: list[dict[str, object]], label_b: str | None) -> str:
    if label_b is None:
        lines = [
            f"# Candidate Parameter Table: {label_a}",
            "",
            "| Parameter | Unit | Value |",
            "|---|---:|---:|",
        ]
        for row in rows:
            lines.append(f"| {row['label']} | {row['unit']} | {_format_number(float(row['value_a']))} |")
        return "\n".join(lines) + "\n"

    lines = [
        f"# Candidate Parameter Comparison: {label_a} vs {label_b}",
        "",
        "| Parameter | Unit | " + label_a + " | " + label_b + " | Delta | Delta % |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        delta_pct = row["delta_pct"]
        delta_pct_text = "n/a" if delta_pct is None else f"{float(delta_pct):.2f}%"
        lines.append(
            "| "
            + f"{row['label']} | {row['unit']} | "
            + f"{_format_number(float(row['value_a']))} | {_format_number(float(row['value_b']))} | "
            + f"{_format_number(float(row['delta_abs']))} | {delta_pct_text} |"
        )
    return "\n".join(lines) + "\n"


def build_parameter_report(
    *,
    candidate_a: Path,
    label_a: str,
    out_dir: Path,
    candidate_b: Path | None = None,
    label_b: str | None = None,
) -> dict[str, object]:
    if (candidate_b is None) != (label_b is None):
        raise ValueError("--candidate-b and --label-b must be provided together")

    out_dir.mkdir(parents=True, exist_ok=True)
    loaded_a = _load_candidate(candidate_a)
    loaded_b = _load_candidate(candidate_b) if candidate_b is not None else None
    rows = _build_rows(loaded_a, loaded_b)
    markdown = _build_markdown(label_a, rows, label_b)

    report = {
        "label_a": label_a,
        "label_b": label_b,
        "candidate_a": str(_resolve_candidate_json(candidate_a).resolve()),
        "candidate_b": str(_resolve_candidate_json(candidate_b).resolve()) if candidate_b is not None else None,
        "rows": rows,
    }
    (out_dir / "parameter_summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (out_dir / "parameter_summary.md").write_text(markdown, encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Build markdown/json tables for candidate parameter sets.")
    parser.add_argument("--candidate-a", required=True)
    parser.add_argument("--label-a", required=True)
    parser.add_argument("--candidate-b", default="")
    parser.add_argument("--label-b", default="")
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    report = build_parameter_report(
        candidate_a=Path(args.candidate_a).expanduser().resolve(),
        label_a=args.label_a,
        candidate_b=Path(args.candidate_b).expanduser().resolve() if args.candidate_b else None,
        label_b=args.label_b or None,
        out_dir=Path(args.out_dir).expanduser().resolve(),
    )
    print(
        json.dumps(
            {
                "ok": True,
                "rows": len(report["rows"]),
                "markdown": str((Path(args.out_dir).expanduser().resolve() / "parameter_summary.md").resolve()),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
