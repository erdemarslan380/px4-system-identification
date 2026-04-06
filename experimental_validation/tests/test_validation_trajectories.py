from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path

from experimental_validation.validation_trajectories import COMMON_LOGGED_START, DEFAULT_VALIDATION_TRAJECTORIES, export_validation_trajectories
from experimental_validation.trajectory_catalog import validation_trajectory_asset_path


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
            source = validation_trajectory_asset_path(DEFAULT_VALIDATION_TRAJECTORIES[0])
            self.assertEqual(traj_path.read_bytes(), source.read_bytes())
            payload = json.loads((out_dir / "validation_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["entries"][0]["traj_id"], DEFAULT_VALIDATION_TRAJECTORIES[0].traj_id)
            self.assertEqual(payload["entries"][0]["sha256"], DEFAULT_VALIDATION_TRAJECTORIES[0].sha256)
            self.assertEqual(payload["entries"][0]["source_sha256"], DEFAULT_VALIDATION_TRAJECTORIES[0].sha256)
            self.assertAlmostEqual(payload["common_logged_start"]["x_m"], COMMON_LOGGED_START[0], places=5)

    def test_export_can_freeze_yaw_and_yawspeed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp) / "trajectories"
            manifest = export_validation_trajectories(
                out_dir,
                entries=[DEFAULT_VALIDATION_TRAJECTORIES[2]],
                freeze_yaw=True,
            )
            payload = json.loads((out_dir / "validation_manifest.json").read_text(encoding="utf-8"))
            self.assertTrue(payload["freeze_yaw"])
            first = manifest["entries"][0]
            sample = struct.unpack("11f", Path(first["path"]).read_bytes()[:44])
            self.assertEqual(sample[9], 0.0)
            self.assertEqual(sample[10], 0.0)
            self.assertNotEqual(first["sha256"], first["source_sha256"])


if __name__ == "__main__":
    unittest.main()
