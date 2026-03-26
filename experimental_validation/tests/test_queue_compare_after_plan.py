from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from experimental_validation import queue_compare_after_plan


class QueueCompareAfterPlanTests(unittest.TestCase):
    def test_wait_for_existing_plan_completion_does_not_launch_missing_plan(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            repo_root = Path(tmp)
            wait_plan = repo_root / "plans" / "suite.yaml"
            wait_plan.parent.mkdir(parents=True, exist_ok=True)
            wait_plan.write_text("name: suite\n", encoding="utf-8")

            manifest_path = repo_root / "plan_runs" / "suite" / "plan_manifest.json"
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            manifest_path.write_text(
                '{"results_root": "/tmp/suite", "tasks": [{"status": "pending"}], "counts": {"pending": 1, "running": 0, "ok": 0, "failed": 0}}',
                encoding="utf-8",
            )

            with patch.object(queue_compare_after_plan, "REPO_ROOT", repo_root), \
                 patch.object(queue_compare_after_plan, "TOOLS_ROOT", repo_root / "Tools" / "optimization"), \
                 patch("experimental_validation.queue_compare_after_plan.manifest_path_for_plan", return_value=manifest_path), \
                 patch("experimental_validation.queue_compare_after_plan.plan_results_root", return_value=manifest_path.parent), \
                 patch("experimental_validation.queue_compare_after_plan.runtime_state_path", return_value=manifest_path.parent / "plan_runtime.json"), \
                 patch("experimental_validation.queue_compare_after_plan.load_json", side_effect=[
                     {"results_root": "/tmp/suite", "tasks": [{"status": "pending"}], "counts": {"pending": 1, "running": 0, "ok": 0, "failed": 0}},
                     None,
                     {"results_root": "/tmp/suite", "tasks": [{"status": "ok"}], "counts": {"pending": 0, "running": 0, "ok": 1, "failed": 0}},
                     None,
                 ]), \
                 patch("experimental_validation.queue_compare_after_plan.is_pid_alive", side_effect=[False, False]), \
                 patch("experimental_validation.queue_compare_after_plan.time.sleep") as sleep_mock, \
                 patch("experimental_validation.queue_compare_after_plan.wait_for_plan", return_value={"results_root": "/tmp/suite"}) as wait_mock:
                result = queue_compare_after_plan.wait_for_existing_plan_completion(
                    wait_plan=wait_plan,
                    serve_dashboard=False,
                    max_resumes=3,
                    poll_s=0.01,
                )

            self.assertEqual(result["results_root"], "/tmp/suite")
            sleep_mock.assert_called()
            wait_mock.assert_called_once()

    def test_wait_then_compare_runs_comparison_pipeline(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            repo_root = Path(tmp)
            wait_plan = repo_root / "plans" / "suite.yaml"
            wait_plan.parent.mkdir(parents=True, exist_ok=True)
            wait_plan.write_text("name: suite\n", encoding="utf-8")
            out_dir = repo_root / "outputs" / "suite"
            logs = [repo_root / "plan_runs" / "suite" / "task" / "identification_traces" / "eval_00000.csv"]
            sdf_reference = {"mass": {"mass_kg": 2.0}}
            reports = {
                "px4_only": {"truth_policy": "none", "profile_counts": {"hover_thrust": 10}, "identified": {"mass": {"mass_kg": 1.0}}, "comparison": {"comparable_metrics": {"mass_kg": {"pct_error": -50.0}}}},
                "telemetry_augmented": {"truth_policy": "telemetry", "profile_counts": {"hover_thrust": 10}, "identified": {"mass": {"mass_kg": 2.0}}, "comparison": {"comparable_metrics": {"mass_kg": {"pct_error": 0.0}}}},
                "truth_assisted": {"truth_policy": "full", "profile_counts": {"hover_thrust": 10}, "identified": {"mass": {"mass_kg": 2.0}}, "comparison": {"comparable_metrics": {"mass_kg": {"pct_error": 0.0}}}},
            }

            with patch.object(queue_compare_after_plan, "REPO_ROOT", repo_root), \
                 patch.object(queue_compare_after_plan, "TOOLS_ROOT", repo_root / "Tools" / "optimization"), \
                 patch("experimental_validation.queue_compare_after_plan.wait_for_existing_plan_completion", return_value={"results_root": str(repo_root / "plan_runs" / "suite")}) as wait_mock, \
                 patch("experimental_validation.queue_compare_after_plan.collect_identification_logs", return_value=logs) as collect_mock, \
                 patch("experimental_validation.queue_compare_after_plan.build_identification_mode_reports", return_value=reports) as reports_mock, \
                 patch("experimental_validation.queue_compare_after_plan.parse_x500_sdf_reference", return_value=sdf_reference) as sdf_mock, \
                 patch("experimental_validation.queue_compare_after_plan.write_comparison_outputs") as write_mock:
                result = queue_compare_after_plan.wait_then_compare(
                    wait_plan=wait_plan,
                    out_dir=out_dir,
                    sdf_model=repo_root / "x500" / "model.sdf",
                    sdf_base_model=repo_root / "x500_base" / "model.sdf",
                    serve_dashboard=False,
                    max_resumes=3,
                )

            self.assertTrue(result["ok"])
            wait_mock.assert_called_once()
            collect_mock.assert_called_once()
            reports_mock.assert_called_once()
            sdf_mock.assert_called_once()
            write_mock.assert_called_once()


if __name__ == "__main__":
    unittest.main()
