from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.build_hitl_review_bundle import build_bundle


class HitlReviewBundleTests(unittest.TestCase):
    def test_build_bundle_copies_logs_and_writes_html(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp = Path(tmp_dir)
            log_root = tmp / "sd_extract"
            tracking_dir = log_root / "tracking_logs"
            identification_dir = log_root / "identification_logs"
            tracking_dir.mkdir(parents=True)
            identification_dir.mkdir(parents=True)

            tracking_csv = tracking_dir / "id_100_run.csv"
            tracking_csv.write_text(
                "\n".join(
                    [
                        "timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller",
                        "0,0,0,-3,0.1,0.0,-2.9,px4_default",
                        "1000000,1,0,-3,0.9,0.1,-3.1,px4_default",
                    ]
                ),
                encoding="utf-8",
            )

            identification_csv = identification_dir / "roll_sweep_r1.csv"
            identification_csv.write_text(
                "\n".join(
                    [
                        "timestamp_us,profile,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller",
                        "0,roll_sweep,0,0,-3,0,0,-3,sysid",
                        "500000,roll_sweep,0,0.2,-3,0.02,0.18,-3.01,sysid",
                    ]
                ),
                encoding="utf-8",
            )

            out_dir = tmp / "review_bundle"
            bundle = build_bundle(log_root, out_dir, max_points=100)

            self.assertEqual(len(bundle["runs"]), 2)
            self.assertTrue((out_dir / "index.html").exists())
            self.assertTrue((out_dir / "summary.json").exists())
            self.assertTrue((out_dir / "raw" / "tracking_logs" / tracking_csv.name).exists())
            self.assertTrue((out_dir / "raw" / "identification_logs" / identification_csv.name).exists())

            summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(len(summary["runs"]), 2)
            self.assertIn("raw/tracking_logs/id_100_run.csv", (out_dir / "index.html").read_text(encoding="utf-8"))
            self.assertIn("Plotly.newPlot('plot3d'", (out_dir / "index.html").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
