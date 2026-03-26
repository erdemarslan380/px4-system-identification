import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_ROOT = REPO_ROOT / "Tools" / "optimization"
sys.path.insert(0, str(TOOL_ROOT))

from plan_watchdog import classify_watchdog_action


class WatchdogTests(unittest.TestCase):
    def test_runner_alive_means_wait(self):
        action, reason = classify_watchdog_action(
            {"tasks": [{}], "counts": {"pending": 0, "running": 1, "failed": 0, "ok": 0}},
            {"pid": 123},
            True,
        )
        self.assertEqual(action, "wait")
        self.assertEqual(reason, "runner_alive")

    def test_unfinished_without_runner_means_resume(self):
        action, reason = classify_watchdog_action(
            {"tasks": [{}, {}], "counts": {"pending": 1, "running": 0, "failed": 0, "ok": 1}},
            {"pid": 0},
            False,
        )
        self.assertEqual(action, "resume")
        self.assertEqual(reason, "unfinished_tasks_without_runner")

    def test_all_ok_means_done(self):
        action, reason = classify_watchdog_action(
            {"tasks": [{}, {}], "counts": {"pending": 0, "running": 0, "failed": 0, "ok": 2}},
            {"pid": 0},
            False,
        )
        self.assertEqual(action, "done")
        self.assertEqual(reason, "all_tasks_ok")


if __name__ == "__main__":
    unittest.main()
