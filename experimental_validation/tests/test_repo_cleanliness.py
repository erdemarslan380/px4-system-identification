from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OVERLAY_ROOT = REPO_ROOT / "overlay" / "src" / "modules"


class RepoCleanlinessTests(unittest.TestCase):
    def test_no_legacy_optimization_tree_exists(self) -> None:
        self.assertFalse((REPO_ROOT / "tools").exists())

    def test_custom_pos_control_has_no_legacy_controller_names(self) -> None:
        content = (OVERLAY_ROOT / "custom_pos_control" / "custom_pos_control.cpp").read_text(encoding="utf-8").lower()
        for token in ("dfbc", "indi", "cmpc", "mpc_pos_controller", "dfbc_attitude_controller", "indi_attitude_controller"):
            self.assertNotIn(token, content)

    def test_trajectory_reader_has_no_legacy_controller_labels(self) -> None:
        content = (OVERLAY_ROOT / "trajectory_reader" / "trajectory_reader.cpp").read_text(encoding="utf-8").lower()
        for token in ('return "dfbc"', 'return "indi"', 'return "cmpc"'):
            self.assertNotIn(token, content)

    def test_repo_docs_do_not_reference_removed_optimization_ui(self) -> None:
        docs = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "system_identification.txt",
            REPO_ROOT / "overlay" / "README.md",
            REPO_ROOT / "experimental_validation" / "README.md",
        ]
        for doc in docs:
            content = doc.read_text(encoding="utf-8").lower()
            for token in ("tools/optimization", "run_simulation_plan", "serve_dashboard", "planner", "dashboard", "review"):
                self.assertNotIn(token, content, msg=f"unexpected legacy token {token} in {doc}")


if __name__ == "__main__":
    unittest.main()
