#!/usr/bin/env python3
"""Run determinism / sim-speed benchmark phases sequentially."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List


def atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)


def find_matching_pids(match_text: str) -> List[int]:
    proc = subprocess.run(
        ["pgrep", "-af", "determinism_speed_benchmark.py"],
        capture_output=True,
        text=True,
        check=False,
    )
    out: List[int] = []
    for line in proc.stdout.splitlines():
        if match_text not in line:
            continue
        try:
            out.append(int(line.split(None, 1)[0]))
        except Exception:
            continue
    return out


def is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def phase_state(phase: Dict[str, Any]) -> str:
    if Path(phase["summary_path"]).exists():
        return "done"
    pids = [pid for pid in find_matching_pids(phase["results_root_arg"]) if is_alive(pid)]
    if pids:
        return "running"
    return "pending"


def start_phase(phase: Dict[str, Any], repo_root: Path) -> int:
    cmd = [sys.executable, "Tools/optimization/determinism_speed_benchmark.py"] + phase["args"]
    log_path = Path(phase["log_path"])
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as handle:
        proc = subprocess.Popen(
            cmd,
            cwd=str(repo_root),
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
    return int(proc.pid)


def write_status(state_path: Path, payload: Dict[str, Any]) -> None:
    atomic_write_json(state_path, payload)


def build_phases(tool_root: Path) -> List[Dict[str, Any]]:
    results_dir = tool_root / "results"
    return [
        {
            "name": "determinism_headless_all",
            "results_root_arg": "Tools/optimization/results/determinism_headless_all",
            "summary_path": str(results_dir / "determinism_headless_all" / "summary.json"),
            "log_path": str(results_dir / "determinism_headless_all.log"),
            "args": [
                "--sim-speeds", "1.0",
                "--modes", "headless",
                "--repeats", "2",
                "--param-set-count", "1",
                "--results-root", "Tools/optimization/results/determinism_headless_all",
            ],
        },
        {
            "name": "determinism_gui_all",
            "results_root_arg": "Tools/optimization/results/determinism_gui_all",
            "summary_path": str(results_dir / "determinism_gui_all" / "summary.json"),
            "log_path": str(results_dir / "determinism_gui_all.log"),
            "args": [
                "--sim-speeds", "1.0",
                "--modes", "gui",
                "--repeats", "2",
                "--param-set-count", "1",
                "--results-root", "Tools/optimization/results/determinism_gui_all",
            ],
        },
        {
            "name": "sim_speed_headless_all",
            "results_root_arg": "Tools/optimization/results/sim_speed_headless_all",
            "summary_path": str(results_dir / "sim_speed_headless_all" / "summary.json"),
            "log_path": str(results_dir / "sim_speed_headless_all.log"),
            "args": [
                "--sim-speeds", "0.5,1.0,2.0",
                "--modes", "headless",
                "--repeats", "1",
                "--param-set-count", "1",
                "--results-root", "Tools/optimization/results/sim_speed_headless_all",
            ],
        },
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description="Queue determinism benchmark phases sequentially.")
    ap.add_argument("--poll-s", type=float, default=15.0)
    args = ap.parse_args()

    tool_root = Path(__file__).resolve().parent
    repo_root = tool_root.parents[1]
    results_dir = tool_root / "results"
    queue_log = results_dir / "queue_determinism_suite.log"
    state_path = results_dir / "queue_determinism_suite.json"
    phases = build_phases(tool_root)

    queue_log.parent.mkdir(parents=True, exist_ok=True)
    with queue_log.open("a", encoding="utf-8") as log:
        log.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] queue started\n")
        log.flush()
        current_pid = None
        current_phase = None
        try:
            while True:
                phase_payloads: List[Dict[str, Any]] = []
                all_done = True
                for phase in phases:
                    state = phase_state(phase)
                    pids = [pid for pid in find_matching_pids(phase["results_root_arg"]) if is_alive(pid)]
                    if state != "done":
                        all_done = False
                    phase_payloads.append({
                        "name": phase["name"],
                        "state": state,
                        "summary_path": phase["summary_path"],
                        "pid": pids[0] if pids else None,
                    })

                write_status(state_path, {
                    "status": "completed" if all_done else "running",
                    "updated_at": time.time(),
                    "phases": phase_payloads,
                    "active_phase": current_phase,
                    "active_pid": current_pid,
                    "poll_s": float(args.poll_s),
                })

                if all_done:
                    log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] queue completed\n")
                    log.flush()
                    return 0

                launched = False
                for phase in phases:
                    state = phase_state(phase)
                    if state == "done":
                        continue
                    if state == "running":
                        current_phase = phase["name"]
                        pids = [pid for pid in find_matching_pids(phase["results_root_arg"]) if is_alive(pid)]
                        current_pid = pids[0] if pids else None
                        break
                    current_pid = start_phase(phase, repo_root)
                    current_phase = phase["name"]
                    log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] started {phase['name']} pid={current_pid}\n")
                    log.flush()
                    launched = True
                    break

                if launched:
                    time.sleep(max(2.0, float(args.poll_s)))
                else:
                    time.sleep(max(5.0, float(args.poll_s)))
        except Exception as exc:
            write_status(state_path, {
                "status": "failed",
                "updated_at": time.time(),
                "phases": [],
                "active_phase": current_phase,
                "active_pid": current_pid,
                "poll_s": float(args.poll_s),
                "error": str(exc),
            })
            log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] queue failed: {exc}\n")
            log.flush()
            raise


if __name__ == "__main__":
    raise SystemExit(main())
