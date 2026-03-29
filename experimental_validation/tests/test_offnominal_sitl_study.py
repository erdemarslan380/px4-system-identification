from __future__ import annotations

import unittest

from experimental_validation.offnominal_sitl_study import (
    OffnominalPerturbation,
    PANEL_GROUPS,
    _trajectory_duration_s,
    offnominal_reference,
)
from experimental_validation.trajectory_catalog import DEFAULT_VALIDATION_TRAJECTORIES


class OffnominalSitlStudyTest(unittest.TestCase):
    def test_offnominal_reference_applies_expected_scales(self) -> None:
        reference = {
            "mass": {"mass_kg": 2.0},
            "inertia": {
                "x": {"inertia_kgm2": 0.02},
                "y": {"inertia_kgm2": 0.03},
                "z": {"inertia_kgm2": 0.04},
            },
            "motor_model": {
                "time_constant_up_s": 0.01,
                "time_constant_down_s": 0.02,
                "max_rot_velocity_radps": 900.0,
                "motor_constant": 8e-6,
                "moment_constant": 0.015,
                "rotor_drag_coefficient": 9e-5,
                "rolling_moment_coefficient": 1e-6,
                "rotor_velocity_slowdown_sim": 10.0,
            },
        }
        perturbation = OffnominalPerturbation()

        report = offnominal_reference(reference, perturbation)

        self.assertAlmostEqual(report["mass_kg"], 1.9)
        self.assertAlmostEqual(report["ixx_kgm2"], 0.019)
        self.assertAlmostEqual(report["iyy_kgm2"], 0.0285)
        self.assertAlmostEqual(report["izz_kgm2"], 0.038)
        self.assertAlmostEqual(report["time_constant_up_s"], 0.0105)
        self.assertAlmostEqual(report["time_constant_down_s"], 0.021)
        self.assertAlmostEqual(report["max_rot_velocity_radps"], 873.0)
        self.assertAlmostEqual(report["rotor_velocity_slowdown_sim"], 10.0)

    def test_duration_helper_reads_shipped_metadata(self) -> None:
        self.assertEqual(_trajectory_duration_s(DEFAULT_VALIDATION_TRAJECTORIES[0]), 23.0)
        self.assertEqual(_trajectory_duration_s(DEFAULT_VALIDATION_TRAJECTORIES[1]), 19.0)

    def test_panel_groups_cover_all_five_cases(self) -> None:
        flattened = [name for group in PANEL_GROUPS for name in group]
        self.assertEqual(flattened, [
            "circle",
            "hairpin",
            "lemniscate",
            "time_optimal_30s",
            "minimum_snap_50s",
        ])


if __name__ == "__main__":
    unittest.main()
