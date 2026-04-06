from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import pandas as pd

from experimental_validation.build_sitl_trajectory_review_bundle import build_bundle


CASES = ("circle", "hairpin", "lemniscate", "time_optimal_30s", "minimum_snap_50s")


def _write_tracking_csv(path: Path, *, offset_x: float, offset_y: float, error_scale: float) -> None:
    rows = []
    for idx in range(30):
        t = float(idx) / 29.0
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


class BuildSitlTrajectoryReviewBundleTests(unittest.TestCase):
    def test_build_bundle_writes_stock_only_review(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stock_root = root / "stock"
            out_dir = root / "review"
            (stock_root / "tracking_logs").mkdir(parents=True)
            for case in CASES:
                _write_tracking_csv(stock_root / "tracking_logs" / f"{case}.csv", offset_x=0.0, offset_y=0.0, error_scale=1.0)

            bundle = build_bundle(
                stock_root=stock_root,
                out_dir=out_dir,
                stock_label="Stock x500 SITL",
                max_points=200,
            )

            self.assertTrue((out_dir / "index.html").exists())
            self.assertTrue((out_dir / "summary.json").exists())
            html = (out_dir / "index.html").read_text(encoding="utf-8")
            self.assertIn("progressSlider", html)
            self.assertIn("Track target", html)
            self.assertIn("Trajectory Switcher", html)
            self.assertIn("caseTabs", html)
            self.assertIn('src="plotly-2.35.2.min.js"', html)
            self.assertIn("Stock x500 SITL", html)
            self.assertTrue((out_dir / "plotly-2.35.2.min.js").exists())
            self.assertEqual(len(bundle["cases"]), 5)

    def test_build_bundle_supports_three_datasets(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stock_root = root / "stock"
            prior_root = root / "prior"
            ident_root = root / "ident"
            out_dir = root / "review"
            for base in (stock_root, prior_root, ident_root):
                (base / "tracking_logs").mkdir(parents=True)
            for idx, case in enumerate(CASES):
                _write_tracking_csv(stock_root / "tracking_logs" / f"{case}.csv", offset_x=0.0, offset_y=0.0, error_scale=1.0 + idx * 0.1)
                _write_tracking_csv(prior_root / "tracking_logs" / f"{case}.csv", offset_x=1.0, offset_y=0.0, error_scale=0.8 + idx * 0.1)
                _write_tracking_csv(ident_root / "tracking_logs" / f"{case}.csv", offset_x=2.0, offset_y=0.0, error_scale=0.6 + idx * 0.1)

            bundle = build_bundle(
                stock_root=stock_root,
                out_dir=out_dir,
                stock_label="Stock x500 SITL",
                compare_root=prior_root,
                compare_label="jMAVSim prior SDF",
                compare_root_2=ident_root,
                compare_label_2="Re-identified from SITL ident",
                max_points=200,
            )

            self.assertEqual(bundle["dataset_labels"], ["Stock x500 SITL", "jMAVSim prior SDF", "Re-identified from SITL ident"])
            case0 = bundle["cases"][0]
            self.assertEqual(len(case0["datasets"]), 3)


if __name__ == "__main__":
    unittest.main()
