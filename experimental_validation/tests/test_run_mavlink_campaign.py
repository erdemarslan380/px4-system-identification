import unittest
from pathlib import Path
from unittest import mock

from examples.run_mavlink_campaign import (
    CAMPAIGN_PARAM_VALUES,
    default_timeout_s,
    prepare_hover,
    wait_for_campaign_completion,
)


class RunMavlinkCampaignTests(unittest.TestCase):
    def test_script_exists(self):
        repo_root = Path(__file__).resolve().parents[2]
        script = repo_root / 'examples' / 'run_mavlink_campaign.py'
        self.assertTrue(script.exists())

    def test_campaign_param_values(self):
        self.assertEqual(CAMPAIGN_PARAM_VALUES['identification_only'], 1)
        self.assertEqual(CAMPAIGN_PARAM_VALUES['full_stack'], 2)
        self.assertEqual(CAMPAIGN_PARAM_VALUES['trajectory_only'], 3)

    def test_timeout_has_safety_margin(self):
        self.assertGreater(default_timeout_s('identification_only'), 256.0)
        self.assertGreater(default_timeout_s('trajectory_only'), 82.0)
        self.assertGreater(default_timeout_s('full_stack'), 338.0)

    def test_wait_for_campaign_completion_returns_when_status_param_reports_complete(self):
        fake_mav = mock.Mock()
        fake_mav.recv_match.return_value = None
        with mock.patch("examples.run_mavlink_campaign.read_param", return_value=2):
            wait_for_campaign_completion(fake_mav, timeout_s=1.0)

    def test_wait_for_campaign_completion_raises_when_status_param_reports_abort(self):
        fake_mav = mock.Mock()
        fake_mav.recv_match.return_value = None
        with mock.patch("examples.run_mavlink_campaign.read_param", return_value=3):
            with self.assertRaises(RuntimeError):
                wait_for_campaign_completion(fake_mav, timeout_s=1.0)

    def test_prepare_hover_can_fall_back_to_blind_settle(self):
        args = mock.Mock()
        args.arm_attempts = 2
        args.hover_z = -3.0
        args.hover_timeout = 10.0
        args.settle_seconds = 3.0
        args.allow_missing_local_position = True
        args.blind_hover_seconds = 7.0
        args.manual_control_mode = 4
        args.pre_offboard_seconds = 1.5
        args.sim_ready_timeout = 12.0
        args.sim_ready_min_local_samples = 3
        with mock.patch("examples.run_mavlink_campaign.prepare_hover_and_anchor") as prepare_mock:
            prepare_hover(mock.Mock(), args)
        prepare_mock.assert_called_once_with(
            mock.ANY,
            hover_z=-3.0,
            hover_timeout=10.0,
            settle_seconds=3.0,
            arm_attempts=2,
            manual_control_mode=4,
            pre_offboard_seconds=1.5,
            sim_ready_timeout=12.0,
            sim_ready_min_local_samples=3,
            allow_missing_local_position=True,
            blind_hover_seconds=7.0,
        )

    def test_prepare_hover_raises_without_fallback(self):
        args = mock.Mock()
        args.arm_attempts = 1
        args.hover_z = -3.0
        args.hover_timeout = 10.0
        args.settle_seconds = 3.0
        args.allow_missing_local_position = False
        args.blind_hover_seconds = 7.0
        args.manual_control_mode = 0
        args.pre_offboard_seconds = 1.0
        args.sim_ready_timeout = 8.0
        args.sim_ready_min_local_samples = 2
        with mock.patch(
            "examples.run_mavlink_campaign.prepare_hover_and_anchor",
            side_effect=TimeoutError("no local position"),
        ):
            with self.assertRaises(TimeoutError):
                prepare_hover(mock.Mock(), args)


if __name__ == '__main__':
    unittest.main()
