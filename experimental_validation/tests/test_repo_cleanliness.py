from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OVERLAY_ROOT = REPO_ROOT / "overlay" / "src" / "modules"
SYNC_SCRIPT = REPO_ROOT / "sync_into_px4_workspace.sh"
PREPARE_SCRIPT = REPO_ROOT / "prepare_identification_workspace.sh"
REFRESH_DEMO_SCRIPT = REPO_ROOT / "examples" / "refresh_demo_assets.sh"
SMOKE_TEST_SCRIPT = REPO_ROOT / "examples" / "run_repo_smoke_test.sh"
HITL_SMOKE_SCRIPT = REPO_ROOT / "examples" / "hitl_shell_smoke.py"
HITL_SMOKE_RUNNER = REPO_ROOT / "examples" / "run_hitl_smoke_test.sh"
JMAVSIM_HITL_RUNNER = REPO_ROOT / "examples" / "start_jmavsim_hitl.sh"
LATEST_CANDIDATE_SCRIPT = REPO_ROOT / "experimental_validation" / "build_latest_x500_candidate.py"
TRAJECTORY_ASSET_DIR = REPO_ROOT / "assets" / "validation_trajectories"


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

    def test_shipped_validation_trajectories_exist(self) -> None:
        self.assertTrue(TRAJECTORY_ASSET_DIR.exists())
        for traj_id in range(100, 105):
            self.assertTrue((TRAJECTORY_ASSET_DIR / f"id_{traj_id}.traj").exists())

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

    def test_operator_helper_scripts_exist(self) -> None:
        self.assertTrue(REFRESH_DEMO_SCRIPT.exists())
        self.assertTrue(SMOKE_TEST_SCRIPT.exists())
        self.assertTrue(HITL_SMOKE_SCRIPT.exists())
        self.assertTrue(HITL_SMOKE_RUNNER.exists())
        self.assertTrue(JMAVSIM_HITL_RUNNER.exists())
        self.assertTrue(LATEST_CANDIDATE_SCRIPT.exists())
        refresh_content = REFRESH_DEMO_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("validation_trajectories.py", refresh_content)
        self.assertIn("generate_sitl_validation_bundle.py", refresh_content)
        smoke_content = SMOKE_TEST_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("test_repo_cleanliness", smoke_content)
        self.assertIn("test_paper_artifacts", smoke_content)
        hitl_smoke_content = HITL_SMOKE_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("SERIAL_CONTROL_FLAG_EXCLUSIVE", hitl_smoke_content)
        self.assertIn("--cmd", hitl_smoke_content)
        self.assertIn("quiet_timeout_s", hitl_smoke_content)
        hitl_runner = HITL_SMOKE_RUNNER.read_text(encoding="utf-8")
        self.assertIn('DEVICE="${1:-/dev/ttyUSB0}"', hitl_runner)
        self.assertIn("custom_pos_control set px4_default", hitl_runner)
        jmavsim_runner = JMAVSIM_HITL_RUNNER.read_text(encoding="utf-8")
        self.assertIn('DEVICE="${2:-/dev/ttyACM0}"', jmavsim_runner)
        self.assertIn('BAUDRATE="${3:-921600}"', jmavsim_runner)
        self.assertIn("PX4_SYSID_HEADLESS", jmavsim_runner)
        self.assertIn("already open", jmavsim_runner)
        self.assertIn("jmavsim_run.sh", jmavsim_runner)
        latest_candidate_content = LATEST_CANDIDATE_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("REQUIRED_PROFILES", latest_candidate_content)
        self.assertIn("sysid_truth_logs", latest_candidate_content)

    def test_primary_docs_cover_cubeorange_build_flow(self) -> None:
        docs = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "system_identification.txt",
            REPO_ROOT / "experimental_validation" / "README.md",
            REPO_ROOT / "examples" / "real_flight_sorties.md",
        ]
        for doc in docs:
            content = doc.read_text(encoding="utf-8")
            self.assertIn("boards/cubepilot/cubeorange/default.px4board", content, msg=f"missing CubeOrange board sync in {doc}")
            self.assertIn("make cubepilot_cubeorange_default", content, msg=f"missing CubeOrange build command in {doc}")
            self.assertIn("make cubepilot_cubeorange_default upload", content, msg=f"missing CubeOrange upload command in {doc}")

        readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4", readme)
        self.assertIn("If you use a different flight controller", readme)

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

    def test_trajectory_reader_uses_explicit_integer_types_for_rc_selector_bounds(self) -> None:
        content = (OVERLAY_ROOT / "trajectory_reader" / "trajectory_reader.cpp").read_text(encoding="utf-8")
        self.assertIn("math::max<int32_t>(0, _param_trj_rc_max_id.get())", content)
        self.assertIn("math::max<int32_t>(1, _rc_selector_max_traj_id + 1)", content)
        self.assertIn("math::constrain<int32_t>(_param_trj_ident_prof.get(), 0, 8)", content)

    def test_trajectory_reader_uses_chunked_identification_log_writes_for_nuttx(self) -> None:
        content = (OVERLAY_ROOT / "trajectory_reader" / "trajectory_reader.cpp").read_text(encoding="utf-8")
        self.assertIn("const auto write_chunk = [&](const char *fmt, auto... values) -> bool {", content)
        self.assertIn("dprintf(_ident_log_fd, fmt, values...)", content)
        self.assertNotIn("char line[1536]", content)

    def test_sync_script_enables_pwm_out_sim_for_hitl(self) -> None:
        content = SYNC_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y", content)

    def test_trajectory_reader_keeps_hold_setpoints_alive_before_offboard(self) -> None:
        content = (OVERLAY_ROOT / "trajectory_reader" / "trajectory_reader.cpp").read_text(encoding="utf-8")
        self.assertIn("publishHoldPositionSetpoint", content)
        self.assertIn("Keep a hold reference streaming while armed so OFFBOARD can latch cleanly", content)
        self.assertIn("even if the operator has already selected TRAJECTORY or IDENTIFICATION mode", content)

    def test_readme_mentions_jmavsim_only_in_hitl_section(self) -> None:
        readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
        hitl_header = "7. HIL/HITL on CubeOrange with jMAVSim"
        self.assertIn(hitl_header, readme)
        before_hitl, after_hitl = readme.split(hitl_header, maxsplit=1)
        self.assertNotIn("jMAVSim", before_hitl)
        self.assertIn("/dev/ttyACM0 921600", after_hitl)
        self.assertIn("QGroundControl", after_hitl)
        self.assertIn("Tools/mavlink_shell.py /dev/ttyUSB0 -b 57600", after_hitl)
        self.assertIn("TRJ_ACTIVE_ID", after_hitl)
        self.assertIn("TRJ_MODE_CMD = 1", after_hitl)


if __name__ == "__main__":
    unittest.main()
