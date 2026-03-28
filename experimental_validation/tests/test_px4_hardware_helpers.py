import importlib.util
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def load_module(relpath: str, name: str):
    path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
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


if __name__ == "__main__":
    unittest.main()
