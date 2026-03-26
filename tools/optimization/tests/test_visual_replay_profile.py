import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_ROOT = REPO_ROOT / 'Tools' / 'optimization'
sys.path.insert(0, str(TOOL_ROOT))

from visual_replay import compute_replay_runtime_profile


class VisualReplayProfileTests(unittest.TestCase):
    def test_headless_replay_keeps_requested_strict_mode(self):
        profile = compute_replay_runtime_profile(
            simulator='gz',
            mission_mode='trajectory',
            strict_requested=True,
            effective_headless=True,
            takeoff_timeout=30.0,
            trajectory_timeout=120.0,
        )
        self.assertTrue(profile['strict_effective'])
        self.assertEqual(profile['takeoff_timeout'], 30.0)
        self.assertEqual(profile['trajectory_timeout'], 120.0)

    def test_gui_identification_replay_relaxes_strict_mode_and_inflates_budgets(self):
        profile = compute_replay_runtime_profile(
            simulator='gz',
            mission_mode='identification',
            strict_requested=True,
            effective_headless=False,
            takeoff_timeout=30.0,
            trajectory_timeout=60.0,
        )
        self.assertFalse(profile['strict_effective'])
        self.assertGreaterEqual(profile['takeoff_timeout'], 60.0)
        self.assertGreaterEqual(profile['trajectory_timeout'], 240.0)
        self.assertIn('relaxed', profile['note'].lower())

    def test_gui_trajectory_replay_keeps_strict_mode_but_gets_larger_budgets(self):
        profile = compute_replay_runtime_profile(
            simulator='gz',
            mission_mode='trajectory',
            strict_requested=True,
            effective_headless=False,
            takeoff_timeout=25.0,
            trajectory_timeout=80.0,
        )
        self.assertTrue(profile['strict_effective'])
        self.assertGreaterEqual(profile['takeoff_timeout'], 60.0)
        self.assertGreaterEqual(profile['trajectory_timeout'], 180.0)
        self.assertEqual(profile['note'], '')


if __name__ == '__main__':
    unittest.main()
