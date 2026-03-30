from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.calibration_restore import (
    format_board_defaults_lines,
    format_param_literal,
    select_calibration_params,
    write_restore_outputs,
)


class CalibrationRestoreTests(unittest.TestCase):
    def test_select_calibration_params_keeps_sensor_and_rc_related_values(self) -> None:
        selected = select_calibration_params({
            "CAL_ACC0_XOFF": 0.1,
            "SENS_BOARD_ROT": 12.0,
            "RC1_TRIM": 1500.0,
            "RC_MAP_ROLL": 1.0,
            "MPC_XY_P": 0.95,
        })
        self.assertIn("CAL_ACC0_XOFF", selected)
        self.assertIn("SENS_BOARD_ROT", selected)
        self.assertIn("RC1_TRIM", selected)
        self.assertIn("RC_MAP_ROLL", selected)
        self.assertNotIn("MPC_XY_P", selected)

    def test_write_restore_outputs_emits_json_params_and_nsh(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            outputs = write_restore_outputs(
                {
                    "CAL_ACC0_XOFF": 0.1,
                    "SENS_BOARD_ROT": 12.0,
                },
                tmp,
            )
            params_text = Path(outputs["params"]).read_text(encoding="utf-8")
            nsh_text = Path(outputs["nsh"]).read_text(encoding="utf-8")
            payload = json.loads(Path(outputs["json"]).read_text(encoding="utf-8"))
            board_defaults_text = Path(outputs["board_defaults"]).read_text(encoding="utf-8")
            self.assertIn("CAL_ACC0_XOFF,0.1", params_text)
            self.assertIn("param set CAL_ACC0_XOFF 0.1", nsh_text)
            self.assertIn("param set-default CAL_ACC0_XOFF 0.1", board_defaults_text)
            self.assertEqual(payload["SENS_BOARD_ROT"], 12.0)

    def test_format_board_defaults_lines_uses_param_set_default(self) -> None:
        lines = format_board_defaults_lines({"CAL_ACC0_XOFF": 0.1, "SENS_BOARD_ROT": 12.0})
        self.assertEqual(lines[0], "#!/bin/sh")
        self.assertIn("param set-default CAL_ACC0_XOFF 0.1", lines)
        self.assertIn("param set-default SENS_BOARD_ROT 12", lines)

    def test_format_param_literal_drops_trailing_dot_zero_for_integers(self) -> None:
        self.assertEqual(format_param_literal(50.0), "50")
        self.assertEqual(format_param_literal(-1.0), "-1")
        self.assertEqual(format_param_literal(0.125), "0.125")


if __name__ == "__main__":
    unittest.main()
