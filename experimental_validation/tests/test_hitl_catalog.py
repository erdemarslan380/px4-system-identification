import unittest

from experimental_validation.hitl_catalog import (
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
