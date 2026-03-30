from __future__ import annotations

import unittest
import struct

from pymavlink import mavutil

from experimental_validation.export_vehicle_params import decode_param_value, format_qgc_parameter_dump
from experimental_validation.qgc_params import parse_qgc_parameter_dump


class ExportVehicleParamsTests(unittest.TestCase):
    def test_decode_param_value_handles_int32_bit_encoding(self) -> None:
        encoded = struct.unpack("<f", struct.pack("<i", 123456789))[0]
        decoded = decode_param_value(encoded, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.assertEqual(decoded, 123456789)

    def test_format_qgc_parameter_dump_round_trips_with_parser(self) -> None:
        payload = format_qgc_parameter_dump(
            {
                "CAL_ACC0_XOFF": (0.1, 9),
                "SENS_BOARD_ROT": (12.0, 6),
            },
            system_id=1,
            component_id=1,
        )
        self.assertIn("1\t1\tCAL_ACC0_XOFF\t0.1\t9", payload)
        parsed = parse_qgc_parameter_dump_str(payload)
        self.assertAlmostEqual(parsed["CAL_ACC0_XOFF"], 0.1)
        self.assertAlmostEqual(parsed["SENS_BOARD_ROT"], 12.0)


def parse_qgc_parameter_dump_str(payload: str) -> dict[str, float]:
    from tempfile import TemporaryDirectory
    from pathlib import Path

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "vehicle.params"
        path.write_text(payload, encoding="utf-8")
        return parse_qgc_parameter_dump(path)


if __name__ == "__main__":
    unittest.main()
