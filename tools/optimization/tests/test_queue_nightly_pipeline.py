import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_ROOT = REPO_ROOT / "Tools" / "optimization"
sys.path.insert(0, str(TOOL_ROOT))

import yaml

from plan_runtime import write_json
from queue_nightly_pipeline import manifest_done_ok, manifest_has_unfinished, normalize_gz_plan_defaults, wait_for_plan
from run_simulation_plan import effective_worker_count


class QueueNightlyPipelineTests(unittest.TestCase):
    def test_manifest_done_ok_requires_all_ok(self):
        manifest = {"tasks": [{}, {}], "counts": {"pending": 0, "running": 0, "failed": 0, "ok": 2}}
        self.assertTrue(manifest_done_ok(manifest))

    def test_manifest_has_unfinished_detects_pending_and_failed(self):
        self.assertTrue(manifest_has_unfinished({"counts": {"pending": 1, "running": 0, "failed": 0}}))
        self.assertTrue(manifest_has_unfinished({"counts": {"pending": 0, "running": 0, "failed": 1}}))
        self.assertFalse(manifest_has_unfinished({"counts": {"pending": 0, "running": 0, "failed": 0}}))

    def test_wait_for_plan_returns_completed_manifest_without_relaunch(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_root = Path(tmp)
            tool_root = repo_root / "Tools" / "optimization"
            tool_root.mkdir(parents=True)
            (tool_root / "results").mkdir(parents=True)
            plan_path = tool_root / "finished.yaml"
            plan_path.write_text(yaml.safe_dump({
                "name": "finished_suite",
                "results_root": "Tools/optimization/plan_runs/finished_suite",
                "tasks": [{"id": "task_a", "controller": "pid", "traj_id": 0}],
            }), encoding="utf-8")
            manifest_path = repo_root / "Tools" / "optimization" / "plan_runs" / "finished_suite" / "plan_manifest.json"
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            manifest = {
                "tasks": [{"task_id": "task_a", "status": "ok"}],
                "counts": {"pending": 0, "running": 0, "failed": 0, "ok": 1},
            }
            write_json(manifest_path, manifest)
            result = wait_for_plan(
                repo_root,
                tool_root,
                plan_path,
                clean_start=False,
                serve_dashboard=False,
                max_resumes=1,
            )
            self.assertEqual(result["counts"]["ok"], 1)
            state_path = tool_root / "results" / "finished_sequence_state.json"
            state = yaml.safe_load(state_path.read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "finished")

    def test_normalize_gz_plan_defaults_clamps_workers_and_takeoff_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            plan_path = Path(tmp) / "gz_plan.yaml"
            plan_path.write_text(yaml.safe_dump({
                "name": "demo",
                "defaults": {
                    "simulator": "gz",
                    "workers": 8,
                    "takeoff_timeout": 30.0,
                },
                "tasks": [{"id": "a", "controller": "pid", "traj_id": 0}],
            }, sort_keys=False), encoding="utf-8")
            normalize_gz_plan_defaults(plan_path)
            loaded = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
            self.assertEqual(loaded["defaults"]["workers"], 1)
            self.assertEqual(float(loaded["defaults"]["takeoff_timeout"]), 45.0)

    def test_effective_worker_count_clamps_gz_tasks_to_one(self):
        self.assertEqual(
            effective_worker_count({
                "workers": 8,
                "global_iters": 8,
                "local_iters": 2,
                "param_dim": 10,
                "simulator": "gz",
            }),
            1,
        )
        self.assertEqual(
            effective_worker_count({
                "workers": 8,
                "global_iters": 8,
                "local_iters": 0,
                "param_dim": 10,
                "simulator": "jmavsim",
            }),
            8,
        )

    def test_wait_for_plan_relaunches_when_manifest_unfinished_but_runner_dead(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_root = Path(tmp)
            tool_root = repo_root / "Tools" / "optimization"
            tool_root.mkdir(parents=True)
            (tool_root / "results").mkdir(parents=True)
            plan_path = tool_root / "unfinished.yaml"
            plan_path.write_text(yaml.safe_dump({
                "name": "unfinished_suite",
                "results_root": "Tools/optimization/plan_runs/unfinished_suite",
                "tasks": [{"id": "task_a", "controller": "pid", "traj_id": 0}],
            }), encoding="utf-8")
            manifest_path = repo_root / "Tools" / "optimization" / "plan_runs" / "unfinished_suite" / "plan_manifest.json"
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            manifest = {
                "tasks": [{"task_id": "task_a", "status": "failed"}],
                "counts": {"pending": 0, "running": 0, "failed": 1, "ok": 0},
            }
            write_json(manifest_path, manifest)

            launch_payload = {
                "pid": 12345,
                "log": str(tool_root / "results" / "fake.log"),
                "manifest": str(manifest_path),
                "results_root": str(manifest_path.parent),
            }

            with patch("queue_nightly_pipeline.is_pid_alive", return_value=False), \
                 patch("queue_nightly_pipeline.launch_detached", return_value=launch_payload), \
                 patch("queue_nightly_pipeline.time.sleep", side_effect=AssertionError("sleep should not be reached")):
                with self.assertRaises(AssertionError):
                    wait_for_plan(
                        repo_root,
                        tool_root,
                        plan_path,
                        clean_start=False,
                        serve_dashboard=False,
                        max_resumes=1,
                    )

            state_path = tool_root / "results" / "unfinished_sequence_state.json"
            state = yaml.safe_load(state_path.read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "running")
            self.assertEqual(state["last_launch"]["pid"], 12345)


if __name__ == "__main__":
    unittest.main()
