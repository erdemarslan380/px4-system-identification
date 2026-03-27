from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path

from experimental_validation.validation_trajectories import COMMON_LOGGED_START, DEFAULT_VALIDATION_TRAJECTORIES, export_validation_trajectories


class ValidationTrajectoriesTests(unittest.TestCase):
    def test_export_writes_manifest_and_binary_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp) / "trajectories"
            manifest = export_validation_trajectories(out_dir)
            self.assertEqual(len(manifest["entries"]), len(DEFAULT_VALIDATION_TRAJECTORIES))
            self.assertTrue((out_dir / "validation_manifest.json").exists())
            first = manifest["entries"][0]
            traj_path = Path(first["path"])
            self.assertTrue(traj_path.exists())
            data = traj_path.read_bytes()
            self.assertGreater(len(data), 0)
            row = struct.unpack("11f", data[:44])
            self.assertAlmostEqual(row[0], COMMON_LOGGED_START[0], places=5)
            self.assertAlmostEqual(row[1], COMMON_LOGGED_START[1], places=5)
            self.assertAlmostEqual(row[2], COMMON_LOGGED_START[2], places=5)
            payload = json.loads((out_dir / "validation_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["entries"][0]["traj_id"], DEFAULT_VALIDATION_TRAJECTORIES[0].traj_id)


if __name__ == "__main__":
    unittest.main()
