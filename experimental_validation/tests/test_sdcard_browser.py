from __future__ import annotations

import unittest
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[2] / "examples" / "sdcard_browser.py"
SPEC = importlib.util.spec_from_file_location("sdcard_browser", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

local_destination_for_remote = MODULE.local_destination_for_remote
normalize_remote_path = MODULE.normalize_remote_path


class SdcardBrowserTests(unittest.TestCase):
    def test_normalize_remote_path_keeps_root(self) -> None:
        self.assertEqual(normalize_remote_path("/fs/microsd", "/fs/microsd"), "/fs/microsd")
        self.assertEqual(normalize_remote_path("/fs/microsd", "/fs/microsd/tracking_logs"), "/fs/microsd/tracking_logs")

    def test_normalize_remote_path_rejects_escape(self) -> None:
        with self.assertRaises(ValueError):
            normalize_remote_path("/fs/microsd", "/fs/microsd/../../etc")

    def test_local_destination_for_remote_preserves_relative_tree(self) -> None:
        root = Path("/tmp/browser_downloads")
        local = local_destination_for_remote(root, "/fs/microsd", "/fs/microsd/tracking_logs/t3r6_15891fec.csv")
        self.assertEqual(local, root / "tracking_logs" / "t3r6_15891fec.csv")


if __name__ == "__main__":
    unittest.main()
