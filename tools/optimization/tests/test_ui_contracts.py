import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_ROOT = REPO_ROOT / "Tools" / "optimization"
ROOTFS = REPO_ROOT / "build" / "px4_sitl_default" / "rootfs"
sys.path.insert(0, str(TOOL_ROOT))

from px4_eval import latest_tracking_log, resolve_eval_logs_and_metrics
from ui_catalog import catalog_payload, load_plan_yaml
from queue_determinism_suite import build_phases, phase_state
from trajectory_utils import trajectory_points


class PlannerContractTests(unittest.TestCase):
    def setUp(self):
        self.html = (TOOL_ROOT / "plan_builder.html").read_text(encoding="utf-8")

    def test_planner_contains_existing_yaml_selector(self):
        self.assertIn('id="existing_plan_select"', self.html)
        self.assertIn("Choosing a saved plan fills the editor automatically.", self.html)

    def test_planner_hides_file_and_port_inputs(self):
        self.assertNotIn('id="plan_file"', self.html)
        self.assertNotIn('id="dashboard_port"', self.html)
        self.assertNotIn('id="yaml_preview"', self.html)

    def test_planner_uses_simple_labels(self):
        self.assertIn("Study Setup", self.html)
        self.assertIn("Start Clean Plan", self.html)
        self.assertIn("Selected Controller Parameters", self.html)
        self.assertIn("Task Order", self.html)
        self.assertIn("Open a saved study or build a new one", self.html)
        self.assertLess(self.html.index("Task Editor"), self.html.index("Runtime Estimate"))
        self.assertLess(self.html.index("Runtime Estimate"), self.html.index("Trajectory Preview"))

    def test_planner_contains_split_timeout_controls(self):
        self.assertIn('id="task_trajectory_timeout_mode"', self.html)
        self.assertIn('id="task_trajectory_timeout_value"', self.html)
        self.assertIn("Auto from trajectory duration", self.html)
        self.assertIn("Maximum Mission Time", self.html)

    def test_planner_contains_identification_controls(self):
        self.assertIn('id="task_mission_mode"', self.html)
        self.assertIn('id="task_ident_profile"', self.html)
        self.assertIn('id="task_base_param_file"', self.html)
        self.assertIn("System Identification", self.html)
        self.assertIn("Vehicle Baseline Parameter Dump", self.html)

    def test_planner_contains_camera_persistence_logic(self):
        self.assertIn("trajectoryCameraCache", self.html)
        self.assertIn("plotly_relayout", self.html)
        self.assertIn("uirevision", self.html)

    def test_planner_contains_equal_axis_trajectory_logic(self):
        self.assertIn("computeTrajectoryAxisRanges", self.html)
        self.assertIn("aspectmode: 'cube'", self.html)
        self.assertIn("tickformat: '.2f'", self.html)

    def test_planner_contains_selected_and_total_runtime_cards(self):
        for element_id in ("stat_task_evals", "stat_task_eta", "stat_tasks", "stat_evals", "stat_eta", "stat_eval"):
            self.assertIn(f'id="{element_id}"', self.html)
        self.assertIn("Selected Task Trials", self.html)
        self.assertIn("Estimated Study Time", self.html)

    def test_planner_generates_plan_filename_from_name(self):
        self.assertIn("function planOutputFilename()", self.html)
        self.assertIn("safePlanStem", self.html)

    def test_planner_uses_parameter_card_grid(self):
        self.assertIn('id="param_grid_body"', self.html)
        self.assertIn("class=\"param-grid\"", self.html)
        self.assertIn("param-card", self.html)
        self.assertIn("Lower Limit", self.html)
        self.assertIn("Upper Limit", self.html)


class CatalogContractTests(unittest.TestCase):
    def test_catalog_exposes_worker_and_sim_speed_limits(self):
        payload = catalog_payload(TOOL_ROOT, ROOTFS)
        limits = payload["ui_limits"]
        self.assertGreaterEqual(limits["workers_max"], limits["workers_min"])
        self.assertGreater(limits["cpu_count"], 0)
        self.assertGreater(limits["sim_speed_factor_max"], limits["sim_speed_factor_min"])

    def test_catalog_exposes_gazebo_as_default_simulator(self):
        payload = catalog_payload(TOOL_ROOT, ROOTFS)
        defaults = payload["ui_defaults"]
        simulators = {item["name"] for item in payload["simulators"]}
        self.assertIn("gz", simulators)
        self.assertIn("jmavsim", simulators)
        self.assertEqual(defaults["simulator"], "gz")
        self.assertTrue(payload["gazebo_models"])
        self.assertTrue(payload["gazebo_worlds"])
        self.assertIn("hover_thrust", payload["identification_profiles"])
        self.assertEqual(defaults["mission_mode"], "trajectory")

    def test_controller_params_include_descriptions(self):
        payload = catalog_payload(TOOL_ROOT, ROOTFS)
        controllers = {item["name"]: item for item in payload["controllers"]}
        self.assertIn("pid", controllers)
        sample = controllers["pid"]["params"][0]
        self.assertIn("description", sample)
        self.assertTrue(sample["description"])

    def test_load_plan_example_has_name_and_tasks(self):
        loaded = load_plan_yaml(TOOL_ROOT, "Tools/optimization/example_plan.yaml")
        self.assertIn("plan", loaded)
        plan = loaded["plan"]
        self.assertTrue(plan.get("name"))
        self.assertGreater(len(plan.get("tasks", [])), 0)
        defaults = plan.get("defaults") or {}
        self.assertEqual(defaults.get("iterations"), 10)
        self.assertEqual(defaults.get("global_iters"), 8)
        self.assertEqual(defaults.get("optimizers"), ["bayes", "random", "anneal"])
        self.assertEqual(defaults.get("simulator"), "gz")

    def test_trajectory_preview_reads_real_points(self):
        points = trajectory_points(ROOTFS, 0, max_points=20)
        self.assertGreater(len(points), 0)
        self.assertIn("x", points[0])
        self.assertIn("y", points[0])
        self.assertIn("z", points[0])
        self.assertIn("t", points[0])

    def test_trajectory_reader_accepts_all_identification_profiles(self):
        source = (REPO_ROOT / "src/modules/trajectory_reader/trajectory_reader.cpp").read_text(encoding="utf-8")
        self.assertIn("_param_trj_ident_prof.get(), 0, 8", source)
        self.assertIn("mass_vertical", source)
        self.assertIn("motor_step", source)


class QueueSuiteContractTests(unittest.TestCase):
    def test_phase_state_reports_done_when_summary_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = root / "summary.json"
            summary.write_text("{}", encoding="utf-8")
            phase = {
                "summary_path": str(summary),
                "results_root_arg": str(root),
            }
            self.assertEqual(phase_state(phase), "done")

    def test_build_phases_names_are_stable(self):
        phases = build_phases(TOOL_ROOT)
        self.assertEqual(
            [phase["name"] for phase in phases],
            ["determinism_headless_all", "determinism_gui_all", "sim_speed_headless_all"],
        )


class EvalContractTests(unittest.TestCase):
    def test_latest_tracking_log_supports_new_trajectory_reader_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = Path(tmp)
            sample = log_dir / "t3r1_deadbeef.csv"
            sample.write_text("timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n", encoding="utf-8")
            found = latest_tracking_log(log_dir, 3, sample.stat().st_mtime)
            self.assertEqual(found, sample)

    def test_eval_runtime_uses_root_tracking_log_dir(self):
        source = (TOOL_ROOT / "px4_eval.py").read_text(encoding="utf-8")
        self.assertIn('rootfs / "tracking_logs"', source)

    def test_identification_eval_can_finish_with_ident_log_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            tracking_dir = root / "tracking_logs"
            identification_dir = root / "identification_logs"
            tracking_dir.mkdir()
            identification_dir.mkdir()
            ident_log = identification_dir / "mass_vertical_r1_deadbeef.csv"
            ident_log.write_text("timestamp_us,profile,thrust_cmd\n1,mass_vertical,0.5\n", encoding="utf-8")
            tracking_log, identification_log, track_rmse, fitness = resolve_eval_logs_and_metrics(
                mission_mode="identification",
                tracking_dir=tracking_dir,
                identification_dir=identification_dir,
                traj_id=0,
                run_begin=ident_log.stat().st_mtime,
                ident_profile="mass_vertical",
                tracking_log_from_loop=None,
                trace_name="sysid",
                energy_term=1.25,
                w_track=1.0,
                w_energy=0.05,
            )
            self.assertIsNone(tracking_log)
            self.assertEqual(identification_log, ident_log)
            self.assertEqual(track_rmse, 0.0)
            self.assertAlmostEqual(fitness, 0.0625)

    def test_trajectory_eval_still_requires_tracking_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            tracking_dir = root / "tracking_logs"
            identification_dir = root / "identification_logs"
            tracking_dir.mkdir()
            identification_dir.mkdir()
            with self.assertRaisesRegex(RuntimeError, "No tracking log found"):
                resolve_eval_logs_and_metrics(
                    mission_mode="trajectory",
                    tracking_dir=tracking_dir,
                    identification_dir=identification_dir,
                    traj_id=0,
                    run_begin=0.0,
                    ident_profile="",
                    tracking_log_from_loop=None,
                    trace_name="px4_default",
                    energy_term=0.0,
                    w_track=1.0,
                    w_energy=0.05,
                )


if __name__ == "__main__":
    unittest.main()
