import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_ROOT = REPO_ROOT / "Tools" / "optimization"
sys.path.insert(0, str(TOOL_ROOT))

import queue_after_plan


class QueueAfterPlanTests(unittest.TestCase):
    def test_wait_then_launches_next_watchdog(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_root = Path(tmp)
            tool_root = repo_root / "Tools" / "optimization"
            tool_root.mkdir(parents=True)
            (tool_root / "results").mkdir(parents=True)
            wait_plan = tool_root / "quick.yaml"
            next_plan = tool_root / "overnight.yaml"
            wait_plan.write_text("name: quick\n", encoding="utf-8")
            next_plan.write_text("name: overnight\n", encoding="utf-8")

            with patch("queue_after_plan.wait_for_plan", return_value={"results_root": str(repo_root / "plan_runs" / "quick")} ) as wait_mock, \
                 patch("queue_after_plan.start_watchdog_detached", return_value={"pid": 99, "log": "watch.log"}) as start_mock:
                result = queue_after_plan.wait_then_start(
                    repo_root,
                    tool_root,
                    wait_plan,
                    next_plan,
                    serve_dashboard=True,
                    max_resumes=5,
                )

            self.assertTrue(result["ok"])
            self.assertEqual(result["watchdog"]["pid"], 99)
            wait_mock.assert_called_once()
            start_mock.assert_called_once()
            state_path = Path(result["state_path"])
            self.assertTrue(state_path.exists())
            state = __import__("json").loads(state_path.read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "next_started")

    def test_module_exports_main(self):
        self.assertTrue(callable(queue_after_plan.main))


if __name__ == "__main__":
    unittest.main()
