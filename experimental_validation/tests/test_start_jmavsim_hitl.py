from __future__ import annotations

import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class StartJmavsimHitlScriptTest(unittest.TestCase):
    def test_script_does_not_exit_when_device_has_no_lsof_holder(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        script = repo_root / "examples" / "start_jmavsim_hitl.sh"

        with tempfile.TemporaryDirectory() as tmpdir:
            px4_root = Path(tmpdir)
            jmavsim_dir = px4_root / "Tools" / "simulation" / "jmavsim"
            jmavsim_dir.mkdir(parents=True)
            runner = jmavsim_dir / "jmavsim_run.sh"
            fake_device = px4_root / "ttyFAKE0"
            fake_device.write_text("", encoding="utf-8")
            runner.write_text(
                "#!/usr/bin/env bash\n"
                "set -euo pipefail\n"
                "echo \"$@\"\n",
                encoding="utf-8",
            )
            runner.chmod(runner.stat().st_mode | stat.S_IXUSR)

            proc = subprocess.run(
                ["bash", str(script), str(px4_root), str(fake_device), "57600"],
                cwd=str(repo_root),
                capture_output=True,
                text=True,
                env={**os.environ, "PX4_SYSID_HEADLESS": "1"},
                check=False,
            )

            self.assertEqual(proc.returncode, 0, msg=proc.stderr)
            self.assertIn(f"-d {fake_device} -b 57600 -r 250", proc.stdout)


if __name__ == "__main__":
    unittest.main()
