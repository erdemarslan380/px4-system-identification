from __future__ import annotations

import inspect
import unittest

import examples.run_hitl_px4_builtin_ident_minimal as built_in_ident


class BuiltInIdentWorkflowTests(unittest.TestCase):
    def test_ident_workflow_uses_takeoff_then_offboard_then_land(self) -> None:
        source = inspect.getsource(built_in_ident.main)
        helper_source = inspect.getsource(built_in_ident.wait_for_takeoff_hover)
        direct_source = inspect.getsource(built_in_ident.observe_direct_offboard_hold)
        land_source = inspect.getsource(built_in_ident.wait_until_landed)

        self.assertIn("send_nav_takeoff(mav)", source)
        self.assertIn("wait_for_takeoff_hover(", source)
        self.assertIn("set_offboard(mav)", source)
        self.assertIn("observe_direct_offboard_hold(", source)
        self.assertIn("set_position_mode(mav)", source)
        self.assertIn("set_land_mode(mav)", source)
        self.assertIn("wait_until_landed(mav, ground_z=baseline_z, timeout_s=args.land_timeout)", source)
        self.assertNotIn("execute_staged_hover_entry(", source)
        self.assertIn("Takeoff hover x=", helper_source)
        self.assertIn("Direct hover x=", direct_source)
        self.assertIn("Vehicle did not land in time", land_source)


if __name__ == "__main__":
    unittest.main()
