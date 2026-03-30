from __future__ import annotations

import unittest

from experimental_validation.report_hil_resources import (
    extract_issue_counts,
    summarize_percentages,
    summarize_ram_usage,
)


class HilResourceReportTests(unittest.TestCase):
    def test_summarize_percentages_handles_empty_input(self) -> None:
        summary = summarize_percentages([])
        self.assertEqual(summary["samples"], 0)
        self.assertIsNone(summary["min_pct"])
        self.assertIsNone(summary["mean_pct"])
        self.assertIsNone(summary["max_pct"])

    def test_summarize_ram_usage_marks_invalid_spikes(self) -> None:
        summary = summarize_ram_usage([24.9, 96657.7, 96657.7])
        self.assertEqual(summary["status"], "invalid_spike_detected")
        self.assertEqual(summary["valid_samples"], 1)
        self.assertAlmostEqual(summary["valid_mean_pct"], 24.9)

    def test_summarize_ram_usage_marks_clean_series_valid(self) -> None:
        summary = summarize_ram_usage([27.5, 28.1, 28.4])
        self.assertEqual(summary["status"], "valid")
        self.assertEqual(summary["valid_samples"], 3)
        self.assertAlmostEqual(summary["valid_max_pct"], 28.4)

    def test_extract_issue_counts_tracks_expected_patterns(self) -> None:
        counts = extract_issue_counts(
            [
                "WARNING [health_and_arming_checks] Preflight Fail: RAM usage too high: 96657.7%",
                "WARNING [load_mon] wq:nav_and_controllers low on stack! (0 bytes left)",
                "ERROR [parameters] verify: failed (-1)",
                "WARNING [failsafe] Failsafe activated",
            ]
        )
        self.assertEqual(counts["ram_usage_too_high"], 1)
        self.assertEqual(counts["low_on_stack"], 1)
        self.assertEqual(counts["parameter_verify_failed"], 1)
        self.assertEqual(counts["failsafe_activated"], 1)
        self.assertEqual(counts["baro_stale"], 0)


if __name__ == "__main__":
    unittest.main()
