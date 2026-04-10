#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES_DIR = REPO_ROOT / "examples"
RESTART_SCRIPT = EXAMPLES_DIR / "restart_hitl_px4_clean_gui.sh"
RUNNER = EXAMPLES_DIR / "run_hitl_px4_builtin_trajectory_minimal.py"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Repeat a HITL hairpin run N times with clean jMAVSim sessions.")
    p.add_argument("--runs", type=int, default=10)
    p.add_argument("--traj-id", type=int, default=100)
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--px4-root", default=str(Path.home() / "PX4-Autopilot-Identification"))
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--hover-z", type=float, default=-3.0)
    p.add_argument("--pre-offboard-seconds", type=float, default=2.0)
    p.add_argument("--max-retries", type=int, default=3)
    return p.parse_args()


def run_step(cmd: list[str], label: str) -> None:
    print(f"\n=== {label} ===", flush=True)
    print(" ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def main() -> int:
    args = parse_args()
    if args.runs < 1:
        raise SystemExit("--runs must be >= 1")

    for run_idx in range(1, args.runs + 1):
        attempt = 0
        while True:
            attempt += 1
            run_step(
                [
                    str(RESTART_SCRIPT),
                    args.px4_root,
                    str(args.baud),
                    args.endpoint,
                ],
                f"Starting clean HIL session (run {run_idx}/{args.runs}, attempt {attempt})",
            )
            try:
                run_step(
                    [
                        sys.executable,
                        str(RUNNER),
                        "--endpoint",
                        args.endpoint,
                        "--traj-id",
                        str(args.traj_id),
                        "--hover-z",
                        str(args.hover_z),
                        "--pre-offboard-seconds",
                        str(args.pre_offboard_seconds),
                    ],
                    f"Hairpin trajectory run {run_idx}/{args.runs}",
                )
                break
            except subprocess.CalledProcessError as exc:
                if attempt >= args.max_retries:
                    raise
                print(
                    f"Run {run_idx} failed with exit code {exc.returncode}; retrying ({attempt}/{args.max_retries})",
                    flush=True,
                )
                time.sleep(2.0)
        time.sleep(2.0)

    print("\nAll HITL hairpin runs completed.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
