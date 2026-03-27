from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.paper_artifacts import COMMON_LOGGED_START, DEFAULT_TRAJECTORIES, _trajectory_reference, generate_paper_artifacts


class PaperArtifactsTests(unittest.TestCase):
    def test_all_validation_trajectories_share_a_common_logged_start_pose(self) -> None:
        for case in DEFAULT_TRAJECTORIES:
            _, x_ref, y_ref, z_ref = _trajectory_reference(case, samples=240)
            self.assertAlmostEqual(float(x_ref[0]), COMMON_LOGGED_START[0], places=6)
            self.assertAlmostEqual(float(y_ref[0]), COMMON_LOGGED_START[1], places=6)
            self.assertAlmostEqual(float(z_ref[0]), COMMON_LOGGED_START[2], places=6)
            self.assertAlmostEqual(float(x_ref[-1]), COMMON_LOGGED_START[0], delta=0.12)
            self.assertAlmostEqual(float(y_ref[-1]), COMMON_LOGGED_START[1], delta=0.12)
            self.assertAlmostEqual(float(z_ref[-1]), COMMON_LOGGED_START[2], delta=0.12)

    def test_generate_paper_artifacts_creates_expected_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "paper_assets"
            summary = generate_paper_artifacts(root, samples_per_traj=160, grid_points=4, seed=7)
            self.assertIn("stage_1_real_flight_validation", summary)
            self.assertIn("stage_2_sitl_statistical_validation", summary)
            self.assertEqual(len(summary["stage_1_real_flight_validation"]["trajectory_overlays"]), 5)
            self.assertGreater(summary["base_blended_twin_score"], 0.0)
            self.assertTrue((root / "paper_validation_summary.json").exists())
            self.assertTrue((root / "figures" / "hairpin_overlay.png").exists())
            self.assertTrue((root / "figures" / "lemniscate_overlay.png").exists())
            self.assertTrue((root / "figures" / "circle_overlay.png").exists())
            self.assertTrue((root / "figures" / "time_optimal_30s_overlay.png").exists())
            self.assertTrue((root / "figures" / "minimum_snap_50s_overlay.png").exists())
            self.assertTrue((root / "figures" / "payload_z_surface.png").exists())
            self.assertTrue((root / "figures" / "payload_z_lines.png").exists())
            self.assertTrue((root / "figures" / "motor_model_surface.png").exists())
            self.assertTrue((root / "figures" / "motor_model_lines.png").exists())
            self.assertTrue((root / "figures" / "parameter_error_bars.png").exists())
            self.assertTrue((root / "figures" / "family_score_bars.png").exists())
            self.assertTrue((root / "figures" / "trajectory_match_scores.png").exists())
            self.assertTrue((root / "data" / "hairpin_synthetic_real.csv").exists())
            self.assertTrue((root / "data" / "payload_z_surface.csv").exists())

            payload = json.loads((root / "paper_validation_summary.json").read_text(encoding="utf-8"))
            self.assertIn("surface_summaries", payload["stage_2_sitl_statistical_validation"])
            self.assertIn("line_plot_summaries", payload["stage_2_sitl_statistical_validation"])
            self.assertIn("base_model_fit_figures", payload)
            self.assertIn("summary_figure", payload["stage_1_real_flight_validation"])
            self.assertGreater(payload["stage_2_sitl_statistical_validation"]["surface_summaries"]["payload_z"]["mean_similarity"], 0.0)


if __name__ == "__main__":
    unittest.main()
