from __future__ import annotations

import itertools
import subprocess
import sys
import unittest
from unittest import mock
from pathlib import Path

from pymavlink import mavutil

from examples.run_hitl_udp_sequence import (
    arm_with_retries,
    decode_param_value,
    encode_param_value,
    heartbeat_is_armed,
    heartbeat_is_offboard,
    hover_target_from_local_position,
    install_pymavlink_message_store_patch,
    local_position_sample_is_reasonable,
    normalize_param_id,
    param_value_matches,
    prepare_hover_and_anchor,
    reset_mode_cmd,
    set_anchor_from_position,
    set_offboard,
    set_position_target_absolute,
    wait_heartbeat,
    wait_for_arm_confirmation,
    wait_for_hover_target_z,
    wait_for_offboard_confirmation,
    wait_for_sim_ready,
)


class RunHitlUdpSequenceScriptTest(unittest.TestCase):
    def test_help_runs_without_pythonpath(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        script = repo_root / "examples" / "run_hitl_udp_sequence.py"
        proc = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)
        self.assertIn("Run one HITL identification profile or trajectory", proc.stdout)
        self.assertIn("--baud BAUD", proc.stdout)

    def test_normalize_param_id_accepts_bytes_and_strings(self) -> None:
        self.assertEqual(normalize_param_id(b"CST_POS_CTRL_EN\x00"), "CST_POS_CTRL_EN")
        self.assertEqual(normalize_param_id("TRJ_MODE_CMD\x00"), "TRJ_MODE_CMD")

    def test_decode_param_value_handles_bytewise_int32(self) -> None:
        raw = 1.401298464324817e-45  # PX4 bytewise encoding for integer 1
        self.assertEqual(decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_INT32), 1)
        self.assertTrue(param_value_matches(raw, 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32))

    def test_encode_param_value_handles_bytewise_int32(self) -> None:
        encoded = encode_param_value(2, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.assertEqual(decode_param_value(encoded, mavutil.mavlink.MAV_PARAM_TYPE_INT32), 2)

    def test_param_value_matches_direct_float_for_int_params(self) -> None:
        self.assertTrue(param_value_matches(1.0, 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32))

    def test_reset_mode_cmd_timeout_is_nonfatal_by_default(self) -> None:
        with mock.patch("examples.run_hitl_udp_sequence.set_param", side_effect=TimeoutError("no confirm")):
            self.assertFalse(reset_mode_cmd(object(), strict=False))

    def test_reset_mode_cmd_timeout_can_be_strict(self) -> None:
        with mock.patch("examples.run_hitl_udp_sequence.set_param", side_effect=TimeoutError("no confirm")):
            with self.assertRaises(TimeoutError):
                reset_mode_cmd(object(), strict=True)

    def test_set_anchor_from_position_writes_three_real32_params(self) -> None:
        with mock.patch("examples.run_hitl_udp_sequence.set_param") as set_param_mock:
            set_anchor_from_position(object(), 1.0, 2.0, -3.0)
        self.assertEqual(set_param_mock.call_count, 3)
        names = [call.args[1] for call in set_param_mock.call_args_list]
        self.assertEqual(names, ["TRJ_ANCHOR_X", "TRJ_ANCHOR_Y", "TRJ_ANCHOR_Z"])

    def test_set_position_target_absolute_writes_five_params(self) -> None:
        with mock.patch("examples.run_hitl_udp_sequence.set_param") as set_param_mock:
            set_position_target_absolute(object(), 1.0, 2.0, -3.0, 0.2)
        self.assertEqual(set_param_mock.call_count, 5)
        names = [call.args[1] for call in set_param_mock.call_args_list]
        self.assertEqual(names, ["TRJ_POS_ABS", "TRJ_POS_X", "TRJ_POS_Y", "TRJ_POS_Z", "TRJ_POS_YAW"])

    def test_hover_target_from_local_position_uses_current_z_as_baseline(self) -> None:
        msg = mock.Mock(x=1.5, y=-2.0, z=-0.4)
        self.assertEqual(hover_target_from_local_position(msg, -3.0), (1.5, -2.0, -3.4))

    def test_wait_for_hover_target_z_uses_absolute_target(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [
            mock.Mock(z=-1.0),
            mock.Mock(z=-2.9),
        ]
        msg = wait_for_hover_target_z(mav, target_z=-3.0, timeout=1.0)
        self.assertAlmostEqual(float(msg.z), -2.9)

    def test_arm_with_retries_retries_until_arm_succeeds(self) -> None:
        with mock.patch("examples.run_hitl_udp_sequence.arm", side_effect=[RuntimeError("busy"), None]) as arm_mock:
            with mock.patch("time.sleep") as sleep_mock:
                arm_with_retries(object(), attempts=2)
        self.assertEqual(arm_mock.call_count, 2)
        sleep_mock.assert_called_once_with(2.0)

    def test_prepare_hover_and_anchor_can_fall_back_to_existing_anchor(self) -> None:
        with (
            mock.patch("examples.run_hitl_udp_sequence.wait_for_sim_ready", return_value=None),
            mock.patch("examples.run_hitl_udp_sequence.set_param"),
            mock.patch("examples.run_hitl_udp_sequence.set_position_target_absolute") as position_target_mock,
            mock.patch("examples.run_hitl_udp_sequence.arm_with_retries"),
            mock.patch("examples.run_hitl_udp_sequence.set_offboard"),
            mock.patch("examples.run_hitl_udp_sequence.request_message_interval"),
            mock.patch("examples.run_hitl_udp_sequence.wait_for_local_position", side_effect=TimeoutError("no local position")),
            mock.patch("examples.run_hitl_udp_sequence.set_anchor_from_position") as anchor_mock,
            mock.patch("time.sleep") as sleep_mock,
        ):
            prepare_hover_and_anchor(
                object(),
                hover_z=-3.0,
                hover_timeout=10.0,
                settle_seconds=2.0,
                arm_attempts=3,
                manual_control_mode=4,
                pre_offboard_seconds=1.5,
                sim_ready_timeout=12.0,
                sim_ready_min_local_samples=3,
                allow_missing_local_position=True,
                blind_hover_seconds=6.0,
            )
        position_target_mock.assert_called_once_with(mock.ANY, 0.0, 0.0, 0.0, 0.0)
        anchor_mock.assert_not_called()
        self.assertEqual(sleep_mock.call_args_list, [mock.call(1.5), mock.call(6.0)])

    def test_prepare_hover_and_anchor_sets_anchor_when_local_position_exists(self) -> None:
        ready_msg = mock.Mock(x=0.0, y=0.1, z=0.0)
        baseline_msg = mock.Mock(x=0.2, y=0.3, z=0.0)
        anchor_msg = mock.Mock(x=1.0, y=2.0, z=-3.0)
        with (
            mock.patch("examples.run_hitl_udp_sequence.wait_for_sim_ready", return_value=ready_msg),
            mock.patch("examples.run_hitl_udp_sequence.set_param"),
            mock.patch("examples.run_hitl_udp_sequence.set_position_target_absolute") as position_target_mock,
            mock.patch("examples.run_hitl_udp_sequence.arm_with_retries"),
            mock.patch("examples.run_hitl_udp_sequence.set_offboard"),
            mock.patch("examples.run_hitl_udp_sequence.request_message_interval"),
            mock.patch(
                "examples.run_hitl_udp_sequence.wait_for_local_position",
                side_effect=[baseline_msg, anchor_msg],
            ),
            mock.patch("examples.run_hitl_udp_sequence.wait_for_hover_target_z", return_value=anchor_msg),
            mock.patch("examples.run_hitl_udp_sequence.set_anchor_from_position") as anchor_mock,
            mock.patch("time.sleep"),
        ):
            prepare_hover_and_anchor(
                object(),
                hover_z=-3.0,
                hover_timeout=10.0,
                settle_seconds=2.0,
                arm_attempts=2,
                manual_control_mode=4,
                pre_offboard_seconds=1.5,
                sim_ready_timeout=12.0,
                sim_ready_min_local_samples=3,
                allow_missing_local_position=False,
                blind_hover_seconds=6.0,
            )
        self.assertEqual(position_target_mock.call_args_list[0].args[1:], (0.0, 0.1, 0.0, 0.0))
        self.assertEqual(position_target_mock.call_args_list[1].args[1:], (0.2, 0.3, -3.0, 0.0))
        anchor_mock.assert_called_once_with(mock.ANY, 1.0, 2.0, -3.0)

    def test_local_position_sample_is_reasonable_rejects_runaway_values(self) -> None:
        self.assertTrue(local_position_sample_is_reasonable(mock.Mock(x=0.1, y=-0.2, z=-3.0)))
        self.assertFalse(local_position_sample_is_reasonable(mock.Mock(x=0.1, y=-0.2, z=-3000.0)))

    def test_wait_for_sim_ready_requires_attitude_and_stable_local_position(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [
            mock.Mock(get_type=mock.Mock(return_value="ATTITUDE")),
            mock.Mock(get_type=mock.Mock(return_value="LOCAL_POSITION_NED"), x=0.0, y=0.0, z=-2.8),
            mock.Mock(get_type=mock.Mock(return_value="LOCAL_POSITION_NED"), x=0.1, y=0.0, z=-2.9),
            mock.Mock(get_type=mock.Mock(return_value="LOCAL_POSITION_NED"), x=0.0, y=0.2, z=-3.0),
        ]
        with mock.patch("examples.run_hitl_udp_sequence.request_message_interval") as request_interval_mock:
            msg = wait_for_sim_ready(mav, timeout=2.0, require_local_position=True, min_local_samples=3)
        self.assertIsNotNone(msg)
        self.assertEqual(request_interval_mock.call_count, 3)

    def test_wait_for_sim_ready_can_pass_without_local_position_when_allowed(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [mock.Mock(get_type=mock.Mock(return_value="ATTITUDE"))]
        with mock.patch("examples.run_hitl_udp_sequence.request_message_interval"):
            msg = wait_for_sim_ready(mav, timeout=1.0, require_local_position=False, min_local_samples=3)
        self.assertIsNone(msg)

    def test_install_pymavlink_message_store_patch_recovers_from_missing_instances(self) -> None:
        install_pymavlink_message_store_patch()

        class FakeMsg:
            _instance_field = "instance"

            def __init__(self, instance: int):
                self.instance = instance
                self._instances = None

        messages = {"DUMMY": FakeMsg(0)}
        msg = FakeMsg(2)

        mavutil.add_message(messages, "DUMMY", msg)

        self.assertIsNotNone(messages["DUMMY"]._instances)
        self.assertIn(2, messages["DUMMY"]._instances)
        self.assertIs(messages["DUMMY"]._instances[2], msg)

    def test_wait_heartbeat_sends_gcs_heartbeat_until_vehicle_replies(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [None, mock.Mock()]
        wait_heartbeat(mav, timeout=1.2)
        self.assertGreaterEqual(mav.mav.heartbeat_send.call_count, 2)

    def test_heartbeat_helpers_decode_arm_and_offboard(self) -> None:
        armed_msg = mock.Mock(base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, custom_mode=(6 << 16))
        self.assertTrue(heartbeat_is_armed(armed_msg))
        self.assertTrue(heartbeat_is_offboard(armed_msg))

    def test_wait_for_arm_confirmation_can_fall_back_to_heartbeat(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [
            mock.Mock(get_type=mock.Mock(return_value="STATUSTEXT"), text="arming"),
            mock.Mock(
                get_type=mock.Mock(return_value="HEARTBEAT"),
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                custom_mode=0,
            ),
        ]
        wait_for_arm_confirmation(mav, timeout=1.0)

    def test_wait_for_arm_confirmation_requires_armed_heartbeat_after_ack(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = itertools.chain([
            mock.Mock(
                get_type=mock.Mock(return_value="COMMAND_ACK"),
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            ),
        ], itertools.repeat(None))
        with self.assertRaisesRegex(TimeoutError, "HEARTBEAT never reported ARMED"):
            wait_for_arm_confirmation(mav, timeout=0.3)

    def test_wait_for_offboard_confirmation_can_fall_back_to_heartbeat(self) -> None:
        mav = mock.Mock()
        mav.recv_match.side_effect = [
            mock.Mock(
                get_type=mock.Mock(return_value="HEARTBEAT"),
                base_mode=0,
                custom_mode=(6 << 16),
            ),
        ]
        wait_for_offboard_confirmation(mav, timeout=1.0)

    def test_set_offboard_sends_set_mode_and_command_long(self) -> None:
        mav = mock.Mock()
        mav.target_system = 1
        mav.target_component = 1
        with mock.patch("examples.run_hitl_udp_sequence.wait_for_offboard_confirmation") as wait_mock:
            set_offboard(mav)
        mav.mav.set_mode_send.assert_called_once()
        mav.mav.command_long_send.assert_called_once()
        wait_mock.assert_called_once_with(mav)


if __name__ == "__main__":
    unittest.main()
