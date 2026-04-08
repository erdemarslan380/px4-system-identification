from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import pandas as pd

from experimental_validation.repeatability_tube_figures import build_repeatability_tube_outputs


def _write_repeat_csv(path: Path, *, lateral_offset: float, vertical_bias: float) -> None:
    rows = []
    for idx in range(40):
        t = float(idx) / 39.0
        ref_x = 2.4 * t
        ref_y = 0.9 * t * (1.0 - t)
        ref_z = -5.0 + 0.1 * t
        pos_x = ref_x + lateral_offset * (0.25 + t)
        pos_y = ref_y + 0.4 * lateral_offset * (1.0 - t)
        pos_z = ref_z + vertical_bias * 0.1
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


class RepeatabilityTubeFigureTests(unittest.TestCase):
    def test_build_repeatability_tube_outputs_writes_summary_and_figures(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = {
                "cases": {
                    "hairpin": {},
                }
            }
            for model_key, base_offset in (
                ("stock", 0.10),
                ("jmavsim_prior_sdf", 0.03),
                ("re_identified_from_sitl_ident", 0.05),
            ):
                runs = []
                for repeat_idx in range(1, 4):
                    csv_path = root / "raw" / model_key / "tracking_logs" / "hairpin" / f"repeat_{repeat_idx:02d}.csv"
                    csv_path.parent.mkdir(parents=True, exist_ok=True)
                    _write_repeat_csv(csv_path, lateral_offset=base_offset + 0.01 * repeat_idx, vertical_bias=0.02 * repeat_idx)
                    runs.append({"csv": str(csv_path), "ok": True})
                summary["cases"]["hairpin"][model_key] = {
                    "label": model_key,
                    "runs": runs,
                }

            result = build_repeatability_tube_outputs(summary=summary, out_root=root / "out", samples=120)

            summary_json = root / "out" / "tube_summary.json"
            self.assertTrue(summary_json.exists())
            self.assertTrue((root / "out" / "tube_summary.md").exists())
            self.assertTrue((root / "out" / "tube_figures" / "hairpin_tube.png").exists())
            self.assertTrue((root / "out" / "tube_figures" / "hairpin_tube.svg").exists())
            self.assertTrue((root / "out" / "tube_stats" / "hairpin.json").exists())

            data = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertIn("hairpin", data["cases"])
            hairpin = data["cases"]["hairpin"]
            self.assertIn("stock", hairpin["models"])
            self.assertIn("prior_vs_reidentified", hairpin)
            self.assertGreaterEqual(hairpin["prior_vs_reidentified"]["mean_iou_pct"], 0.0)
            self.assertEqual(len(hairpin["models"]["stock"]["centerline"]["x"]), 120)
            self.assertEqual(result["summary_json"], str(summary_json.resolve()))


if __name__ == "__main__":
    unittest.main()
