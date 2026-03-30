import os
import tempfile
import unittest
from pathlib import Path

from examples.mavlink_serial_hub import make_endpoint


class MavlinkSerialHubTests(unittest.TestCase):
    def test_make_endpoint_creates_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            link = Path(td) / "sim.pty"
            endpoint = make_endpoint("sim", link)
            try:
                self.assertTrue(link.is_symlink())
                self.assertEqual(os.readlink(link), endpoint.slave_path)
                self.assertTrue(Path(endpoint.slave_path).exists())
                self.assertEqual(endpoint.slave_fd, -1)
            finally:
                os.close(endpoint.master_fd)
                link.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
