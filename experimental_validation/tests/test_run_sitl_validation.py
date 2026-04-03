from __future__ import annotations

import inspect
import unittest

import experimental_validation.run_sitl_validation as sitl_validation


class RunSitlValidationFlowTests(unittest.TestCase):
    def test_validation_uses_fixed_yaw_offboard_climb_handover(self) -> None:
        source = inspect.getsource(sitl_validation.run_validation_model)
        self.assertIn('session.set_mode("POSCTL")', source)
        self.assertIn("arm_with_retries(", source)
        self.assertIn("observe_armed_ground_hold(", source)
        self.assertIn("send_position_target_local_ned(", source)
        self.assertIn("set_offboard(mav)", source)
        self.assertIn('set_param(mav, "TRJ_POS_ABS", 0', source)
        self.assertIn("wait_for_ground_quiet(", source)
        self.assertIn("set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])", source)
        self.assertIn('set_param(mav, "CST_POS_CTRL_EN", 1', source)
        self.assertIn('label="Direct hover"', source)
        self.assertIn('label="Custom hold"', source)
        self.assertNotIn('session.expect("Ready for takeoff!"', source)
        self.assertNotIn("prepare_hover_and_anchor(", source)
        self.assertNotIn("execute_staged_hover_entry(", source)
        self.assertNotIn('session.send_no_wait("commander takeoff")', source)
        self.assertLess(source.index("observe_armed_ground_hold("), source.index("set_offboard(mav)"))
        self.assertLess(source.index("set_offboard(mav)"), source.index('set_param(mav, "CST_POS_CTRL_EN", 1'))

    def test_validation_starts_built_in_trajectory_via_params(self) -> None:
        source = inspect.getsource(sitl_validation.run_validation_model)
        self.assertIn('set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])', source)
        self.assertIn('set_param(mav, "TRJ_ACTIVE_ID", entry.traj_id', source)
        self.assertIn('set_param(mav, "TRJ_MODE_CMD", 1', source)
        self.assertNotIn('session.send_no_wait("commander mode offboard")', source)
        self.assertIn('session.set_mode("POSCTL")', source)

    def test_validation_uses_session_start_retry_and_optional_esc_override(self) -> None:
        model_source = inspect.getsource(sitl_validation.run_validation_model)
        main_source = inspect.getsource(sitl_validation.main)
        retry_source = inspect.getsource(sitl_validation._start_session_with_retries)

        self.assertIn("_start_session_with_retries(", model_source)
        self.assertIn('if sitl_esc_max is not None:', model_source)
        self.assertIn('_apply_x500_esc_scaling(session, min_value=sitl_esc_min, max_value=sitl_esc_max)', model_source)
        self.assertIn('time.sleep(SITL_START_RETRY_DELAY_SECONDS)', retry_source)
        self.assertIn('--sitl-esc-max', main_source)
        self.assertIn('--sitl-esc-min', main_source)


if __name__ == "__main__":
    unittest.main()
