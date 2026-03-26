from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.calibration_restore import select_calibration_params, write_restore_outputs


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
            self.assertIn("CAL_ACC0_XOFF,0.1", params_text)
            self.assertIn("param set CAL_ACC0_XOFF 0.1", nsh_text)
            self.assertEqual(payload["SENS_BOARD_ROT"], 12.0)


if __name__ == "__main__":
    unittest.main()
