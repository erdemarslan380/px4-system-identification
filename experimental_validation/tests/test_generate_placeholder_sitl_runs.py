from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.generate_placeholder_sitl_runs import generate_placeholder_sitl_runs


class GeneratePlaceholderSitlRunsTests(unittest.TestCase):
    def test_generator_creates_two_run_roots_and_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_root = Path(tmp) / "stage1_inputs"
            payload = generate_placeholder_sitl_runs(out_root, candidate_json=None, samples_per_traj=160, seed=9)
            self.assertTrue((out_root / "stock_sitl_proxy" / "run_manifest.json").exists())
            self.assertTrue((out_root / "digital_twin_sitl" / "run_manifest.json").exists())
            self.assertTrue((out_root / "placeholder_bundle_manifest.json").exists())

            stock_manifest = json.loads((out_root / "stock_sitl_proxy" / "run_manifest.json").read_text(encoding="utf-8"))
            twin_manifest = json.loads((out_root / "digital_twin_sitl" / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(len(stock_manifest["results"]), 5)
            self.assertEqual(len(twin_manifest["results"]), 5)
            self.assertEqual(payload["stock_root"], str((out_root / "stock_sitl_proxy").resolve()))
            self.assertEqual(payload["twin_root"], str((out_root / "digital_twin_sitl").resolve()))


if __name__ == "__main__":
    unittest.main()
