from __future__ import annotations

import subprocess
import sys
import unittest
from unittest import mock
from pathlib import Path

from pymavlink import mavutil

from examples.run_hitl_udp_sequence import (
    decode_param_value,
    encode_param_value,
    normalize_param_id,
    param_value_matches,
    prepare_hover_and_anchor,
    reset_mode_cmd,
    set_anchor_from_position,
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

    def test_prepare_hover_and_anchor_can_fall_back_to_existing_anchor(self) -> None:
        with (
            mock.patch("examples.run_hitl_udp_sequence.arm"),
            mock.patch("examples.run_hitl_udp_sequence.set_offboard"),
            mock.patch("examples.run_hitl_udp_sequence.wait_for_hover", side_effect=TimeoutError("no local position")),
            mock.patch("examples.run_hitl_udp_sequence.set_anchor_from_position") as anchor_mock,
            mock.patch("time.sleep") as sleep_mock,
        ):
            prepare_hover_and_anchor(
                object(),
                hover_z=-3.0,
                hover_timeout=10.0,
                settle_seconds=2.0,
                allow_missing_local_position=True,
                blind_hover_seconds=6.0,
            )
        anchor_mock.assert_not_called()
        sleep_mock.assert_called_once_with(6.0)

    def test_prepare_hover_and_anchor_sets_anchor_when_local_position_exists(self) -> None:
        anchor_msg = mock.Mock(x=1.0, y=2.0, z=-3.0)
        with (
            mock.patch("examples.run_hitl_udp_sequence.arm"),
            mock.patch("examples.run_hitl_udp_sequence.set_offboard"),
            mock.patch("examples.run_hitl_udp_sequence.wait_for_hover"),
            mock.patch("examples.run_hitl_udp_sequence.wait_for_local_position", return_value=anchor_msg),
            mock.patch("examples.run_hitl_udp_sequence.set_anchor_from_position") as anchor_mock,
            mock.patch("time.sleep"),
        ):
            prepare_hover_and_anchor(
                object(),
                hover_z=-3.0,
                hover_timeout=10.0,
                settle_seconds=2.0,
                allow_missing_local_position=False,
                blind_hover_seconds=6.0,
            )
        anchor_mock.assert_called_once_with(mock.ANY, 1.0, 2.0, -3.0)


if __name__ == "__main__":
    unittest.main()
