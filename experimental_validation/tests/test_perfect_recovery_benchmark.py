from __future__ import annotations

import unittest

from experimental_validation.perfect_recovery_benchmark import run_perfect_recovery_benchmark


class PerfectRecoveryBenchmarkTests(unittest.TestCase):
    def test_perfect_recovery_benchmark_matches_reference_closely(self) -> None:
        payload = run_perfect_recovery_benchmark()
        metrics = payload["comparison"]["comparable_metrics"]

        self.assertLess(abs(metrics["mass_kg"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["ixx_kgm2"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["iyy_kgm2"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["izz_kgm2"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["time_constant_up_s"]["pct_error"]), 0.5)
        self.assertLess(abs(metrics["time_constant_down_s"]["pct_error"]), 0.5)
        self.assertLess(abs(metrics["motor_constant"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["moment_constant"]["pct_error"]), 0.1)
        self.assertLess(abs(metrics["rotor_velocity_slowdown_sim"]["pct_error"]), 0.1)


if __name__ == "__main__":
    unittest.main()
