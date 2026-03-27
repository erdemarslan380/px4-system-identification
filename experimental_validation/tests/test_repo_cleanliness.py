from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OVERLAY_ROOT = REPO_ROOT / "overlay" / "src" / "modules"
SYNC_SCRIPT = REPO_ROOT / "sync_into_px4_workspace.sh"
PREPARE_SCRIPT = REPO_ROOT / "prepare_identification_workspace.sh"


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
            for token in ("tools/optimization", "run_simulation_plan", "serve_dashboard", "all_controllers_quick_validation", "overnight_bayes_viable"):
                self.assertNotIn(token, content, msg=f"unexpected legacy token {token} in {doc}")

    def test_primary_docs_use_real_default_paths_instead_of_path_to_placeholders(self) -> None:
        docs = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "system_identification.txt",
            REPO_ROOT / "experimental_validation" / "README.md",
            REPO_ROOT / "examples" / "visual_sitl_walkthrough.md",
            REPO_ROOT / "examples" / "real_flight_sorties.md",
        ]
        for doc in docs:
            content = doc.read_text(encoding="utf-8")
            self.assertNotIn("/path/to/", content, msg=f"placeholder path remained in {doc}")

    def test_overlay_includes_multi_trajectory_message(self) -> None:
        msg_path = REPO_ROOT / "overlay" / "msg" / "MultiTrajectorySetpoint.msg"
        self.assertTrue(msg_path.exists())
        content = msg_path.read_text(encoding="utf-8")
        self.assertIn("MAX_HORIZON", content)
        self.assertIn("position_x", content)

    def test_sync_script_patches_msg_cmakelists(self) -> None:
        content = SYNC_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("patch_msg_cmake", content)
        self.assertIn("MultiTrajectorySetpoint.msg", content)

    def test_workspace_scripts_protect_the_shared_px4_tree(self) -> None:
        sync_content = SYNC_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("PX4_SYSID_ALLOW_SHARED_TREE", sync_content)
        self.assertIn("PX4-Autopilot-Identification", sync_content)
        self.assertTrue(PREPARE_SCRIPT.exists())
        prepare_content = PREPARE_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("PX4-Autopilot-Identification", prepare_content)
        self.assertIn("git clone https://github.com/PX4/PX4-Autopilot.git --recursive", prepare_content)

    def test_modules_use_module_yaml_instead_of_legacy_param_sources(self) -> None:
        modules = ("custom_pos_control", "trajectory_reader")
        for module_name in modules:
            module_dir = OVERLAY_ROOT / module_name
            self.assertTrue((module_dir / "module.yaml").exists(), msg=f"missing module.yaml for {module_name}")
            self.assertFalse((module_dir / f"{module_name}_params.c").exists(), msg=f"legacy params.c still present for {module_name}")

            cmake = (module_dir / "CMakeLists.txt").read_text(encoding="utf-8")
            self.assertIn("MODULE_CONFIG", cmake)
            self.assertIn("module.yaml", cmake)
            self.assertNotIn("_params.c", cmake)

    def test_identification_mode_is_not_forced_back_to_position_on_offboard_entry(self) -> None:
        content = (OVERLAY_ROOT / "trajectory_reader" / "trajectory_reader.cpp").read_text(encoding="utf-8")
        self.assertNotIn("Always reset to POSITION mode on OFFBOARD entry", content)
        self.assertIn("Preserve the selected workflow across OFFBOARD entry.", content)


if __name__ == "__main__":
    unittest.main()
