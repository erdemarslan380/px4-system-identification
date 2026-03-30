import unittest

from experimental_validation.hitl_catalog import (
    campaign_expected_duration_s,
    campaign_ident_profiles,
    campaign_trajectory_ids,
    identification_duration_s,
    identification_profile_index,
    trajectory_duration_s,
)


class HitlCatalogTests(unittest.TestCase):
    def test_identification_profile_indices(self):
        self.assertEqual(identification_profile_index("hover_thrust"), 0)
        self.assertEqual(identification_profile_index("motor_step"), 8)

    def test_identification_durations(self):
        self.assertEqual(identification_duration_s("drag_x"), 30.0)
        self.assertEqual(identification_duration_s("mass_vertical"), 36.0)

    def test_trajectory_durations(self):
        self.assertEqual(trajectory_duration_s(100), 23.0)
        self.assertEqual(trajectory_duration_s(104), 14.0)

    def test_identification_campaign_order(self):
        self.assertEqual(
            campaign_ident_profiles("identification_only"),
            [
                "hover_thrust",
                "mass_vertical",
                "roll_sweep",
                "pitch_sweep",
                "yaw_sweep",
                "drag_x",
                "drag_y",
                "drag_z",
                "motor_step",
            ],
        )
        self.assertEqual(campaign_trajectory_ids("identification_only"), [])

    def test_full_stack_campaign_contains_five_validation_trajectories(self):
        self.assertEqual(campaign_trajectory_ids("full_stack"), [100, 101, 102, 103, 104])

    def test_trajectory_only_campaign_contains_only_validation_trajectories(self):
        self.assertEqual(campaign_ident_profiles("trajectory_only"), [])
        self.assertEqual(campaign_trajectory_ids("trajectory_only"), [100, 101, 102, 103, 104])

    def test_campaign_duration_includes_return_buffer(self):
        self.assertEqual(campaign_expected_duration_s("identification_only", return_buffer_s=0.0), 256.0)
        self.assertEqual(campaign_expected_duration_s("trajectory_only", return_buffer_s=0.0), 82.0)
        self.assertEqual(campaign_expected_duration_s("full_stack", return_buffer_s=0.0), 338.0)
