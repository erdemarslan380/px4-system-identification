from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.build_hitl_repeatability_review_bundle import build_bundle


class HitlRepeatabilityReviewBundleTests(unittest.TestCase):
    def test_build_bundle_writes_review_assets(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp = Path(tmp_dir)
            tracking_dir = tmp / "tracking_logs"
            tracking_dir.mkdir(parents=True)

            for run_idx, offset in enumerate((0.0, 0.05, -0.03), start=1):
                (tracking_dir / f"run_{run_idx:02d}.csv").write_text(
                    "\n".join(
                        [
                            "timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller",
                            f"0,0,0,-5,{offset},0,-5,mpc",
                            f"100000,1,0,-5,{1 + offset},0.02,-5.02,mpc",
                            f"200000,2,0,-5,{2 + offset},-0.01,-5.01,mpc",
                        ]
                    ),
                    encoding="utf-8",
                )

            out_dir = tmp / "review"
            bundle = build_bundle(
                tracking_dir=tracking_dir,
                out_dir=out_dir,
                case_name="hairpin",
                label="HITL repeatability",
                max_runs=3,
                samples=60,
                max_points=120,
            )

            self.assertEqual(len(bundle["runs"]), 3)
            self.assertTrue((out_dir / "index.html").exists())
            self.assertTrue((out_dir / "summary.json").exists())
            self.assertTrue((out_dir / "figures" / "hairpin_tube.png").exists())
            self.assertTrue((out_dir / "figures" / "hairpin_tube.svg").exists())
            self.assertTrue((out_dir / "raw" / "tracking_logs" / "run_01.csv").exists())
            self.assertTrue((out_dir / "sanitized" / "run_01.csv").exists())

            summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["case_name"], "hairpin")
            self.assertEqual(len(summary["runs"]), 3)
            self.assertIn("reference_overlap", summary)
            self.assertIn("mean_iou_pct", summary["reference_overlap"])
            self.assertIn("data:text/csv;base64,", (out_dir / "index.html").read_text(encoding="utf-8"))
            self.assertIn("Hairpin tube view", (out_dir / "index.html").read_text(encoding="utf-8"))
            self.assertIn("Overlap comparison module", (out_dir / "index.html").read_text(encoding="utf-8"))
            self.assertIn("progressSliderBottom", (out_dir / "index.html").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
