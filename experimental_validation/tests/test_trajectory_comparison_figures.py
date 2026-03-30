from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import pandas as pd

from experimental_validation.trajectory_comparison_figures import build_comparison_figures


CASES = ("circle", "hairpin", "lemniscate", "time_optimal_30s", "minimum_snap_50s")


def _write_tracking_csv(path: Path, *, offset_x: float, offset_y: float, error_scale: float) -> None:
    rows = []
    for idx in range(24):
        t = float(idx) / 23.0
        ref_x = offset_x + 2.0 * t
        ref_y = offset_y + 0.5 * t
        ref_z = -3.0 - 0.2 * t
        pos_x = ref_x + error_scale * 0.1
        pos_y = ref_y - error_scale * 0.05
        pos_z = ref_z + error_scale * 0.02
        rows.append(
            {
                "timestamp_us": idx * 20000,
                "ref_x": ref_x,
                "ref_y": ref_y,
                "ref_z": ref_z,
                "pos_x": pos_x,
                "pos_y": pos_y,
                "pos_z": pos_z,
                "controller": "px4_default",
            }
        )
    pd.DataFrame(rows).to_csv(path, index=False)


class TrajectoryComparisonFiguresTests(unittest.TestCase):
    def test_build_comparison_figures_writes_grouped_outputs_and_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stock_root = root / "stock"
            compare_root = root / "compare"
            out_dir = root / "out"
            (stock_root / "tracking_logs").mkdir(parents=True)
            (compare_root / "tracking_logs").mkdir(parents=True)

            for case_index, case in enumerate(CASES):
                _write_tracking_csv(
                    stock_root / "tracking_logs" / f"{case}.csv",
                    offset_x=0.0,
                    offset_y=0.0,
                    error_scale=1.0 + case_index * 0.1,
                )
                _write_tracking_csv(
                    compare_root / "tracking_logs" / f"{case}.csv",
                    offset_x=10.0,
                    offset_y=20.0,
                    error_scale=1.5 + case_index * 0.1,
                )

            summary = build_comparison_figures(
                stock_root=stock_root,
                compare_root=compare_root,
                compare_label="Real flight baseline PID",
                out_dir=out_dir,
            )

            self.assertTrue((out_dir / "group_1_circle_hairpin_lemniscate.png").exists())
            self.assertTrue((out_dir / "group_2_time_optimal_minimum_snap.png").exists())
            self.assertTrue((out_dir / "comparison_summary.json").exists())
            self.assertEqual(summary["compare_label"], "Real flight baseline PID")
            self.assertIn("circle", summary["cases"])


if __name__ == "__main__":
    unittest.main()
