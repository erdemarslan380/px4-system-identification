from __future__ import annotations

import unittest

from experimental_validation.reference_models import default_candidate_identified, default_x500_reference
from experimental_validation.reference_models import X500_FAMILY_COMPOSITE_V1, X500_FAMILY_COMPOSITE_V2
from experimental_validation.twin_metrics import (
    build_blended_twin_score,
    build_blended_twin_score_from_values,
    flatten_identified_metrics,
    flatten_reference_metrics,
)


class TwinMetricsTests(unittest.TestCase):
    def test_perfect_match_scores_near_one_hundred(self) -> None:
        ref = flatten_reference_metrics(default_x500_reference())
        payload = build_blended_twin_score_from_values(ref, ref)
        self.assertGreater(payload["score"], 99.99)
        self.assertTrue(all(value > 99.99 for value in payload["family_scores"].values()))

    def test_example_candidate_scores_lower_than_perfect(self) -> None:
        ref = flatten_reference_metrics(default_x500_reference())
        candidate = flatten_identified_metrics(default_candidate_identified())
        payload = build_blended_twin_score_from_values(candidate, ref)
        self.assertLess(payload["score"], 99.0)
        self.assertIn("inertia", payload["family_scores"])
        self.assertLess(payload["family_scores"]["inertia"], payload["family_scores"]["motor_coefficients"])

    def test_current_builtin_candidate_is_not_worse_than_legacy_v1(self) -> None:
        ref = flatten_reference_metrics(default_x500_reference())
        legacy = build_blended_twin_score_from_values(flatten_identified_metrics(X500_FAMILY_COMPOSITE_V1), ref)
        current = build_blended_twin_score_from_values(flatten_identified_metrics(X500_FAMILY_COMPOSITE_V2), ref)
        self.assertGreaterEqual(current["score"], legacy["score"])

    def test_blended_score_accepts_comparable_metric_payload(self) -> None:
        comparable = {
            "mass_kg": {"identified": 2.1, "reference": 2.0, "abs_error": 0.1, "pct_error": 5.0},
            "ixx_kgm2": {"identified": 0.022, "reference": 0.021, "abs_error": 0.001, "pct_error": 4.76190476},
            "iyy_kgm2": {"identified": 0.022, "reference": 0.022, "abs_error": 0.0, "pct_error": 0.0},
            "izz_kgm2": {"identified": 0.041, "reference": 0.040, "abs_error": 0.001, "pct_error": 2.5},
            "time_constant_up_s": {"identified": 0.011, "reference": 0.010, "abs_error": 0.001, "pct_error": 10.0},
            "time_constant_down_s": {"identified": 0.021, "reference": 0.020, "abs_error": 0.001, "pct_error": 5.0},
            "max_rot_velocity_radps": {"identified": 910.0, "reference": 900.0, "abs_error": 10.0, "pct_error": 1.11111111},
            "motor_constant": {"identified": 1.25e-5, "reference": 1.20e-5, "abs_error": 0.05e-5, "pct_error": 4.16666667},
            "moment_constant": {"identified": 0.021, "reference": 0.020, "abs_error": 0.001, "pct_error": 5.0},
            "rotor_drag_coefficient": {"identified": 7.2e-5, "reference": 7.0e-5, "abs_error": 0.2e-5, "pct_error": 2.85714286},
            "rolling_moment_coefficient": {"identified": 1.1e-6, "reference": 1.0e-6, "abs_error": 0.1e-6, "pct_error": 10.0},
            "rotor_velocity_slowdown_sim": {"identified": 7.8, "reference": 8.0, "abs_error": -0.2, "pct_error": -2.5},
        }
        payload = build_blended_twin_score(comparable)
        self.assertGreater(payload["score"], 0.0)
        self.assertLessEqual(payload["score"], 100.0)
        self.assertIn("motor_dynamics", payload["family_scores"])


if __name__ == "__main__":
    unittest.main()
