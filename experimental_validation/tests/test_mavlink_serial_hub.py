import os
import tempfile
import unittest
from unittest import mock
from pathlib import Path

from examples.mavlink_serial_hub import PtyEndpoint, SerialHub, make_endpoint


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

    def test_handle_pty_ignores_eio_without_crashing(self) -> None:
        hub = SerialHub.__new__(SerialHub)
        hub._serial = mock.Mock()
        endpoint = PtyEndpoint(
            name="ctrl",
            master_fd=123,
            slave_fd=-1,
            slave_path="/dev/pts/fake",
            link_path=Path("/tmp/fake"),
        )
        with mock.patch("examples.mavlink_serial_hub.os.read", side_effect=OSError(5, "Input/output error")):
            hub._handle_pty(endpoint)
        hub._serial.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
