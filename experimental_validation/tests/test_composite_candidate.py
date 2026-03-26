from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.composite_candidate import (
    build_composite_identified,
    load_identified_output,
    write_composite_outputs,
)
from experimental_validation.compare_with_sdf import parse_x500_sdf_reference


class CompositeCandidateTests(unittest.TestCase):
    def test_build_composite_identified_uses_requested_families(self) -> None:
        mass_source = {
            "source_name": "mass.json",
            "mass": {"mass_kg": 2.0},
            "thrust_scale": {"thrust_scale_n_per_cmd": 30.0},
            "inertia": {"x": {"inertia_kgm2": 9.0}, "y": {"inertia_kgm2": 9.0}, "z": {"inertia_kgm2": 9.0}},
            "drag": {"x": {"coefficient": 9.0}, "y": {"coefficient": 9.0}, "z": {"coefficient": 9.0}},
            "motor_model": {"time_constant_up_s": {"value": 9.0}},
            "warnings": ["mass-warning"],
        }
        inertia_source = {
            "source_name": "inertia.json",
            "mass": {"mass_kg": 9.0},
            "thrust_scale": {"thrust_scale_n_per_cmd": 9.0},
            "inertia": {"x": {"inertia_kgm2": 0.02}, "y": {"inertia_kgm2": 0.03}, "z": {"inertia_kgm2": 0.04}},
            "drag": {"x": {"coefficient": 9.0}, "y": {"coefficient": 9.0}, "z": {"coefficient": 9.0}},
            "motor_model": {"time_constant_up_s": {"value": 9.0}},
            "warnings": ["inertia-warning"],
        }
        drag_source = {
            "source_name": "drag.json",
            "mass": {"mass_kg": 9.0},
            "thrust_scale": {"thrust_scale_n_per_cmd": 9.0},
            "inertia": {"x": {"inertia_kgm2": 9.0}, "y": {"inertia_kgm2": 9.0}, "z": {"inertia_kgm2": 9.0}},
            "drag": {"x": {"coefficient": 0.11}, "y": {"coefficient": 0.12}, "z": {"coefficient": 0.13}},
            "motor_model": {"time_constant_up_s": {"value": 9.0}},
            "warnings": ["drag-warning"],
        }
        motor_source = {
            "source_name": "motor.json",
            "mass": {"mass_kg": 9.0},
            "thrust_scale": {"thrust_scale_n_per_cmd": 9.0},
            "inertia": {"x": {"inertia_kgm2": 9.0}, "y": {"inertia_kgm2": 9.0}, "z": {"inertia_kgm2": 9.0}},
            "drag": {"x": {"coefficient": 9.0}, "y": {"coefficient": 9.0}, "z": {"coefficient": 9.0}},
            "motor_model": {"time_constant_up_s": {"value": 0.0125}, "time_constant_down_s": {"value": 0.025}},
            "warnings": ["motor-warning"],
        }

        composite = build_composite_identified(
            mass_source=mass_source,
            inertia_source=inertia_source,
            drag_source=drag_source,
            motor_source=motor_source,
        )

        self.assertEqual(composite["mass"]["mass_kg"], 2.0)
        self.assertEqual(composite["thrust_scale"]["thrust_scale_n_per_cmd"], 30.0)
        self.assertEqual(composite["inertia"]["x"]["inertia_kgm2"], 0.02)
        self.assertEqual(composite["drag"]["z"]["coefficient"], 0.13)
        self.assertEqual(composite["motor_model"]["time_constant_down_s"]["value"], 0.025)
        self.assertIn("mass_source: mass-warning", composite["warnings"])
        self.assertIn("motor_source: motor-warning", composite["warnings"])

    def test_load_and_write_composite_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            src = tmp_path / "identified_parameters.json"
            payload = {
                "mass": {"mass_kg": 2.0},
                "thrust_scale": {"thrust_scale_n_per_cmd": 30.0},
                "inertia": {
                    "x": {"inertia_kgm2": 0.02166666666666667},
                    "y": {"inertia_kgm2": 0.02166666666666667},
                    "z": {"inertia_kgm2": 0.04000000000000001},
                },
                "drag": {"x": {"coefficient": 0.1}, "y": {"coefficient": 0.1}, "z": {"coefficient": 0.1}},
                "motor_model": {
                    "time_constant_up_s": {"value": 0.0125},
                    "time_constant_down_s": {"value": 0.025},
                    "max_rot_velocity_radps": {"value": 1000.0},
                    "motor_constant": {"value": 8.54858e-06},
                    "moment_constant": {"value": 0.016},
                    "rotor_drag_coefficient": {"value": 8.06428e-05},
                    "rolling_moment_coefficient": {"value": 1.0e-06},
                    "rotor_velocity_slowdown_sim": {"value": 10.0},
                },
                "warnings": [],
            }
            src.write_text(json.dumps(payload), encoding="utf-8")
            loaded = load_identified_output(src)
            self.assertEqual(loaded["mass"]["mass_kg"], 2.0)

            reference = parse_x500_sdf_reference(
                Path("/home/earsub/px4-custom/Tools/simulation/gz/models/x500/model.sdf"),
                Path("/home/earsub/px4-custom/Tools/simulation/gz/models/x500_base/model.sdf"),
            )
            out_dir = tmp_path / "out"
            write_composite_outputs(
                out_dir,
                identified=payload,
                sdf_reference=reference,
                source_paths={"mass": "a", "inertia": "b", "drag": "c", "motor_model": "d"},
            )
            self.assertTrue((out_dir / "sdf_comparison.json").exists())
            self.assertTrue((out_dir / "candidate_inertial.sdf.xml").exists())
            comparison = json.loads((out_dir / "sdf_comparison.json").read_text())
            self.assertAlmostEqual(comparison["comparable_metrics"]["mass_kg"]["pct_error"], 0.0, places=9)
