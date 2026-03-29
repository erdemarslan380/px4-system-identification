from __future__ import annotations

import tempfile
import unittest
import importlib.util
from pathlib import Path
from unittest import mock

MODULE_PATH = Path(__file__).resolve().parents[2] / "examples" / "pull_sdcard_logs_over_mavftp.py"
SPEC = importlib.util.spec_from_file_location("pull_sdcard_logs_over_mavftp", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

cleanup_known_holders = MODULE.cleanup_known_holders
download_logs = MODULE.download_logs
download_file_with_retries = MODULE.download_file_with_retries
expected_size_from_entry = MODULE.expected_size_from_entry
ftp_error_label = MODULE.ftp_error_label
local_file_matches = MODULE.local_file_matches
list_remote_matching_files = MODULE.list_remote_matching_files
list_port_holders = MODULE.list_port_holders
list_remote_csvs = MODULE.list_remote_csvs
parse_remote_groups = MODULE.parse_remote_groups
wait_for_port_free = MODULE.wait_for_port_free
wait_for_heartbeat = MODULE.wait_for_heartbeat
write_pull_report = MODULE.write_pull_report


class _Entry:
    def __init__(self, name: str, is_dir: bool = False, size_b: int | None = None) -> None:
        self.name = name
        self.is_dir = is_dir
        self.size_b = size_b


class _Result:
    def __init__(self, return_code: int) -> None:
        self.return_code = return_code


class _FakeFTP:
    def __init__(self, directory_map: dict[str, list[_Entry]], failures_before_success: int = 0) -> None:
        self.directory_map = directory_map
        self.list_result: list[_Entry] = []
        self.downloads: list[tuple[str, str]] = []
        self.failures_before_success = failures_before_success
        self.get_calls = 0
        self.pending_download: tuple[str, str] | None = None

    def cmd_list(self, args: list[str]) -> _Result:
        remote_dir = args[0]
        self.list_result = list(self.directory_map.get(remote_dir, []))
        return _Result(0)

    def cmd_get(self, args: list[str]) -> object:
        remote_path, local_path = args
        self.downloads.append((remote_path, local_path))
        self.pending_download = (remote_path, local_path)
        return object()

    def process_ftp_reply(self, _operation_name: str, timeout: float = 0) -> _Result:
        del timeout
        self.get_calls += 1
        if self.get_calls <= self.failures_before_success:
            return _Result(1)

        assert self.pending_download is not None
        remote_path, local_path = self.pending_download
        entry_name = Path(remote_path).name
        expected_size = None
        for entries in self.directory_map.values():
            for entry in entries:
                if entry.name == entry_name:
                    expected_size = entry.size_b
                    break
        payload = b"timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n"
        if expected_size is not None and expected_size > 0:
            repeats = (expected_size // len(payload)) + 1
            data = (payload * repeats)[:expected_size]
        else:
            data = payload
        Path(local_path).write_bytes(data)
        return _Result(0)


class _FakeMaster:
    def __init__(self, heartbeats_before_success: int = 1):
        self.remaining = heartbeats_before_success

    def recv_match(self, type=None, blocking=False, timeout=None):
        del type, blocking, timeout
        if self.remaining > 0:
            self.remaining -= 1
            return object()
        return None


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

    def test_list_remote_matching_files_supports_multiple_suffixes(self) -> None:
        ftp = _FakeFTP(
            {
                "/fs/microsd/log/2026-03-28": [
                    _Entry("a.ulg"),
                    _Entry("b.csv"),
                    _Entry("c.txt"),
                    _Entry("nested", is_dir=True),
                ]
            }
        )
        result = list_remote_matching_files(ftp, "/fs/microsd/log/2026-03-28", suffixes=(".ulg", ".csv"))
        self.assertEqual(result, ["a.ulg", "b.csv"])

    def test_download_logs_creates_expected_layout(self) -> None:
        ftp = _FakeFTP(
            {
                "/fs/microsd/tracking_logs": [_Entry("t100.csv", size_b=64)],
                "/fs/microsd/identification_logs": [_Entry("hover.csv", size_b=96)],
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
            self.assertEqual(tracking_path.stat().st_size, 64)
            self.assertEqual(identification_path.stat().st_size, 96)
            self.assertEqual(
                ftp.downloads,
                [
                    ("/fs/microsd/tracking_logs/t100.csv", str(tracking_path)),
                    ("/fs/microsd/identification_logs/hover.csv", str(identification_path)),
                ],
            )

    def test_parse_remote_groups_defaults_and_custom_groups(self) -> None:
        self.assertEqual(parse_remote_groups([]), MODULE.REMOTE_GROUPS)
        self.assertEqual(
            parse_remote_groups(["ulog=/fs/microsd/log/2026-03-28", "csv=/fs/microsd/tracking_logs"]),
            {"ulog": "/fs/microsd/log/2026-03-28", "csv": "/fs/microsd/tracking_logs"},
        )

    def test_download_file_retries_after_failure(self) -> None:
        ftp = _FakeFTP(
            {"/fs/microsd/tracking_logs": [_Entry("run.csv", size_b=80)]},
            failures_before_success=1,
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            local_path = Path(tmp_dir) / "run.csv"
            result = download_file_with_retries(
                ftp,
                "/fs/microsd/tracking_logs/run.csv",
                local_path,
                expected_size=80,
                retries=2,
                timeout=5.0,
            )

            self.assertEqual(result, local_path)
            self.assertTrue(local_path.exists())
            self.assertEqual(local_path.stat().st_size, 80)
            self.assertEqual(ftp.get_calls, 2)

    def test_expected_size_from_entry_handles_missing_size(self) -> None:
        self.assertEqual(expected_size_from_entry(_Entry("a.csv", size_b=12)), 12)
        self.assertIsNone(expected_size_from_entry(_Entry("a.csv")))

    def test_ftp_error_label_maps_known_code(self) -> None:
        self.assertEqual(ftp_error_label(73), "RemoteReplyTimeout")

    def test_local_file_matches_uses_expected_size_when_available(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "run.ulg"
            path.write_bytes(b"12345")
            self.assertTrue(local_file_matches(path, 5))
            self.assertFalse(local_file_matches(path, 4))
            self.assertTrue(local_file_matches(path, None))

    def test_write_pull_report_persists_summary_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            report_path = write_pull_report(
                Path(tmp_dir),
                pulled={"ulog": ["a.ulg"]},
                skipped={"ulog": ["b.ulg"]},
                failed={"ulog": [{"file": "c.ulg", "error": "timeout"}]},
            )
            payload = report_path.read_text(encoding="utf-8")
            self.assertIn('"pulled"', payload)
            self.assertIn('"a.ulg"', payload)
            self.assertIn('"skipped"', payload)
            self.assertIn('"failed"', payload)

    def test_wait_for_heartbeat_returns_none_on_timeout(self) -> None:
        master = _FakeMaster(heartbeats_before_success=0)
        self.assertIsNone(wait_for_heartbeat(master, timeout=0.2))

    def test_wait_for_heartbeat_returns_message(self) -> None:
        master = _FakeMaster(heartbeats_before_success=1)
        self.assertIsNotNone(wait_for_heartbeat(master, timeout=0.5))

    def test_list_port_holders_parses_lsof_output(self) -> None:
        completed = mock.Mock(stdout="123\n456\n123\n", returncode=0)
        with mock.patch.object(MODULE.subprocess, "run", return_value=completed):
            self.assertEqual(list_port_holders("/dev/ttyACM0"), [123, 456])

    def test_wait_for_port_free_observes_busy_then_free(self) -> None:
        with mock.patch.object(MODULE, "list_port_holders", side_effect=[[11], [], []]):
            self.assertTrue(wait_for_port_free("/dev/ttyACM0", timeout=0.5))

    def test_cleanup_known_holders_calls_pkill_for_patterns(self) -> None:
        def fake_run(args, capture_output=False, text=False, check=False):
            del capture_output, text, check
            if args[:2] == ["pgrep", "-f"]:
                if "jmavsim_run.jar" in args[2]:
                    return mock.Mock(stdout="111\n222\n")
                return mock.Mock(stdout="")
            return mock.Mock(stdout="")

        with mock.patch.object(MODULE.subprocess, "run", side_effect=fake_run) as run_mock:
            cleanup_known_holders(current_pid=222)
        issued = [call.args[0] for call in run_mock.call_args_list]
        self.assertIn(["pgrep", "-f", "jmavsim_run.jar"], issued)
        self.assertIn(["kill", "-9", "111"], issued)
        self.assertNotIn(["kill", "-9", "222"], issued)


if __name__ == "__main__":
    unittest.main()
