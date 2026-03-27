from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.sitl_validation_artifacts import generate_sitl_validation_artifacts


def _write_tracking_csv(path: Path, *, scale: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n"]
    for idx in range(80):
        t = idx * 20000
        ref_x = 0.02 * idx
        ref_y = 0.01 * idx
        ref_z = -3.0
        pos_x = ref_x + scale * 0.02
        pos_y = ref_y - scale * 0.015
        pos_z = ref_z + scale * 0.01
        lines.append(f"{t},{ref_x:.6f},{ref_y:.6f},{ref_z:.6f},{pos_x:.6f},{pos_y:.6f},{pos_z:.6f},px4_default\n")
    path.write_text("".join(lines), encoding="utf-8")


class SitlValidationArtifactsTests(unittest.TestCase):
    def test_generate_from_real_stage1_logs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stock_root = root / "stock_sitl_proxy"
            twin_root = root / "digital_twin_sitl"
            case_names = ["hairpin", "lemniscate", "circle", "time_optimal_30s", "minimum_snap_50s"]
            stock_results = []
            twin_results = []
            for idx, name in enumerate(case_names, start=100):
                stock_log = stock_root / "tracking_logs" / f"{name}.csv"
                twin_log = twin_root / "tracking_logs" / f"{name}.csv"
                _write_tracking_csv(stock_log, scale=1.0)
                _write_tracking_csv(twin_log, scale=0.3)
                stock_results.append({"traj_id": idx, "name": name, "tracking_log": str(stock_log)})
                twin_results.append({"traj_id": idx, "name": name, "tracking_log": str(twin_log)})

            (stock_root / "run_manifest.json").write_text(json.dumps({"results": stock_results}, indent=2), encoding="utf-8")
            (twin_root / "run_manifest.json").write_text(json.dumps({"results": twin_results}, indent=2), encoding="utf-8")

            out_dir = root / "paper_assets"
            summary = generate_sitl_validation_artifacts(
                out_dir,
                stock_root=stock_root,
                twin_root=twin_root,
                candidate_json=None,
                grid_points=4,
                seed=5,
                display_jitter_std_m=0.005,
            )
            self.assertIn("stage_1_real_flight_validation", summary)
            self.assertEqual(len(summary["stage_1_real_flight_validation"]["trajectory_overlays"]), 5)
            self.assertTrue((out_dir / "figures" / "hairpin_overlay.png").exists())
            self.assertTrue((out_dir / "figures" / "trajectory_rmse_summary.png").exists())
            self.assertTrue((out_dir / "data" / "hairpin_stock_sitl_proxy.csv").exists())
            self.assertIn("stock x500 SITL", summary["stage_1_real_flight_validation"]["note"])


if __name__ == "__main__":
    unittest.main()
