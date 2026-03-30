import unittest
from pathlib import Path
from unittest import mock

from examples.run_mavlink_campaign import (
    CAMPAIGN_PARAM_VALUES,
    default_timeout_s,
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

    def test_timeout_has_safety_margin(self):
        self.assertGreater(default_timeout_s('identification_only'), 256.0)
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


if __name__ == '__main__':
    unittest.main()
