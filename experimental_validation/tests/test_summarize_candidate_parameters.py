from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.summarize_candidate_parameters import build_parameter_report


def _candidate_json(*, mass: float, motor_constant: float) -> dict:
    return {
        "mass": {"mass_kg": mass},
        "thrust_scale": {"thrust_scale_n_per_cmd": 16.0},
        "inertia": {
            "x": {"inertia_kgm2": 0.005},
            "y": {"inertia_kgm2": 0.005},
            "z": {"inertia_kgm2": 0.009},
        },
        "drag": {
            "x": {"coefficient": 0.01},
            "y": {"coefficient": 0.01},
            "z": {"coefficient": 0.01},
        },
        "motor_model": {
            "time_constant_up_s": {"value": 0.005},
            "time_constant_down_s": {"value": 0.005},
            "max_rot_velocity_radps": {"value": 1000.0},
            "motor_constant": {"value": motor_constant},
            "moment_constant": {"value": 0.0125},
            "rotor_drag_coefficient": {"value": 8.06428e-05},
            "rolling_moment_coefficient": {"value": 1.0e-06},
            "rotor_velocity_slowdown_sim": {"value": 10.0},
        },
    }


class SummarizeCandidateParametersTests(unittest.TestCase):
    def test_build_parameter_report_writes_single_candidate_table(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "candidate"
            candidate.mkdir()
            (candidate / "identified_parameters.json").write_text(
                json.dumps(_candidate_json(mass=0.8, motor_constant=4.0e-06)),
                encoding="utf-8",
            )
            out_dir = root / "out"

            report = build_parameter_report(
                candidate_a=candidate,
                label_a="jMAVSim prior SDF",
                out_dir=out_dir,
            )

            self.assertTrue((out_dir / "parameter_summary.md").exists())
            self.assertTrue((out_dir / "parameter_summary.json").exists())
            markdown = (out_dir / "parameter_summary.md").read_text(encoding="utf-8")
            self.assertIn("Candidate Parameter Table: jMAVSim prior SDF", markdown)
            self.assertEqual(report["label_b"], None)

    def test_build_parameter_report_writes_comparison_table(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate_a = root / "candidate_a"
            candidate_b = root / "candidate_b"
            candidate_a.mkdir()
            candidate_b.mkdir()
            (candidate_a / "identified_parameters.json").write_text(
                json.dumps(_candidate_json(mass=0.8, motor_constant=4.0e-06)),
                encoding="utf-8",
            )
            (candidate_b / "identified_parameters.json").write_text(
                json.dumps(_candidate_json(mass=0.85, motor_constant=4.5e-06)),
                encoding="utf-8",
            )
            out_dir = root / "out"

            report = build_parameter_report(
                candidate_a=candidate_a,
                label_a="jMAVSim prior SDF",
                candidate_b=candidate_b,
                label_b="Re-identified from SITL ident",
                out_dir=out_dir,
            )

            markdown = (out_dir / "parameter_summary.md").read_text(encoding="utf-8")
            self.assertIn("Re-identified from SITL ident", markdown)
            self.assertIn("Delta %", markdown)
            self.assertEqual(len(report["rows"]), 16)


if __name__ == "__main__":
    unittest.main()
