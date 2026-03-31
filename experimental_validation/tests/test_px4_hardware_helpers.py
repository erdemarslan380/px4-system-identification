import importlib.util
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def load_module(relpath: str, name: str):
    path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class TestPx4HardwareHelpers(unittest.TestCase):
    def test_upload_helper_defaults(self):
        mod = load_module("examples/upload_cubeorange_firmware.py", "upload_helper")
        self.assertEqual(mod.DEFAULT_WORKSPACE.name, "PX4-Autopilot-Identification")
        self.assertEqual(mod.DEFAULT_FIRMWARE.name, "cubepilot_cubeorange_default.px4")

    def test_nsh_runner_defaults(self):
        mod = load_module("examples/px4_nsh_runner.py", "nsh_runner")
        self.assertEqual(mod.DEFAULT_PORT, "/dev/ttyACM0")
        self.assertEqual(mod.DEFAULT_BAUD, 57600)
        self.assertEqual(mod.PROMPT, "nsh>")

    def test_serial_hub_parser_decodes_heartbeat(self):
        mod = load_module("examples/mavlink_serial_hub.py", "serial_hub")
        parser = mod.make_mavlink_parser()

        mav = mod.mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 42
        mav.srcComponent = 7
        packet = mav.heartbeat_encode(
            mod.mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mod.mavutil.mavlink.MAV_AUTOPILOT_PX4,
            0,
            0,
            0,
        ).pack(mav)

        decoded = None
        for byte in packet:
            decoded = parser.parse_char(bytes([byte])) or decoded

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.get_type(), "HEARTBEAT")
        self.assertEqual(decoded.get_srcSystem(), 42)
        self.assertEqual(decoded.get_srcComponent(), 7)

    def test_serial_hub_sanitizes_bytes_for_json(self):
        mod = load_module("examples/mavlink_serial_hub.py", "serial_hub_json")
        payload = {"raw": b"\x01\x02", "nested": (1, 2)}
        self.assertEqual(mod.sanitize_for_json(payload), {"raw": [1, 2], "nested": [1, 2]})


if __name__ == "__main__":
    unittest.main()
