from __future__ import annotations

import tempfile
import unittest
import importlib.util
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[2] / "examples" / "pull_sdcard_logs_over_mavftp.py"
SPEC = importlib.util.spec_from_file_location("pull_sdcard_logs_over_mavftp", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

download_logs = MODULE.download_logs
list_remote_csvs = MODULE.list_remote_csvs


class _Entry:
    def __init__(self, name: str, is_dir: bool = False) -> None:
        self.name = name
        self.is_dir = is_dir


class _Result:
    def __init__(self, return_code: int) -> None:
        self.return_code = return_code


class _FakeFTP:
    def __init__(self, directory_map: dict[str, list[_Entry]]) -> None:
        self.directory_map = directory_map
        self.list_result: list[_Entry] = []
        self.downloads: list[tuple[str, str]] = []

    def cmd_list(self, args: list[str]) -> _Result:
        remote_dir = args[0]
        self.list_result = list(self.directory_map.get(remote_dir, []))
        return _Result(0)

    def cmd_get(self, args: list[str]) -> object:
        remote_path, local_path = args
        self.downloads.append((remote_path, local_path))
        Path(local_path).write_text("timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n", encoding="utf-8")
        return object()


class PullSdcardLogsOverMavftpTests(unittest.TestCase):
    def test_list_remote_csvs_filters_non_csv_and_directories(self) -> None:
        ftp = _FakeFTP(
            {
                "/fs/microsd/tracking_logs": [
                    _Entry("run_b.csv"),
                    _Entry("ignore.txt"),
                    _Entry("nested", is_dir=True),
                    _Entry("run_a.csv"),
                ]
            }
        )

        result = list_remote_csvs(ftp, "/fs/microsd/tracking_logs")
        self.assertEqual(result, ["run_a.csv", "run_b.csv"])

    def test_download_logs_creates_expected_layout(self) -> None:
        ftp = _FakeFTP(
            {
                "/fs/microsd/tracking_logs": [_Entry("t100.csv")],
                "/fs/microsd/identification_logs": [_Entry("hover.csv")],
            }
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            pulled = download_logs(ftp, Path(tmp_dir))

            tracking_path = Path(tmp_dir) / "tracking_logs" / "t100.csv"
            identification_path = Path(tmp_dir) / "identification_logs" / "hover.csv"

            self.assertEqual(pulled["tracking_logs"], [tracking_path])
            self.assertEqual(pulled["identification_logs"], [identification_path])
            self.assertTrue(tracking_path.exists())
            self.assertTrue(identification_path.exists())
            self.assertEqual(
                ftp.downloads,
                [
                    ("/fs/microsd/tracking_logs/t100.csv", str(tracking_path)),
                    ("/fs/microsd/identification_logs/hover.csv", str(identification_path)),
                ],
            )


if __name__ == "__main__":
    unittest.main()
