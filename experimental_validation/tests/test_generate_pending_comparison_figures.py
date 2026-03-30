from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from experimental_validation.generate_pending_comparison_figures import build_pending_group_figures


class GeneratePendingComparisonFiguresTests(unittest.TestCase):
    def test_build_pending_group_figures_writes_grouped_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            figures = build_pending_group_figures(
                out_dir=out_dir,
                compare_label="HIL-identified SITL",
                pending_reason="Awaiting first complete HIL identification campaign.",
            )

            self.assertTrue((out_dir / "group_1_circle_hairpin_lemniscate.png").exists())
            self.assertTrue((out_dir / "group_2_time_optimal_minimum_snap.png").exists())
            self.assertTrue((out_dir / "comparison_summary.json").exists())
            self.assertIn("group_1_circle_hairpin_lemniscate", figures)


if __name__ == "__main__":
    unittest.main()
