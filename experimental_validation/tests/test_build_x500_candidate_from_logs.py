from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path

from experimental_validation.build_x500_candidate_from_logs import latest_profile_logs, latest_truth_csvs


class BuildX500CandidateFromLogsTests(unittest.TestCase):
    def test_latest_profile_logs_selects_newest_file_per_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_root = Path(tmp)
            older = ident_root / "hover_thrust_r1_old.csv"
            newer = ident_root / "hover_thrust_r2_new.csv"
            older.write_text("x\n", encoding="utf-8")
            time.sleep(0.01)
            newer.write_text("x\n", encoding="utf-8")

            selected, missing = latest_profile_logs(ident_root)

            self.assertEqual(selected["hover_thrust"], newer.resolve())
            self.assertIn("mass_vertical", missing)

    def test_latest_truth_csvs_returns_latest_when_present(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            truth_root = Path(tmp)
            older = truth_root / "truth_1.csv"
            newer = truth_root / "truth_2.csv"
            older.write_text("x\n", encoding="utf-8")
            time.sleep(0.01)
            newer.write_text("x\n", encoding="utf-8")

            selected = latest_truth_csvs(truth_root)

            self.assertEqual(selected, [newer.resolve()])


if __name__ == "__main__":
    unittest.main()
