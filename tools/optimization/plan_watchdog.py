#!/usr/bin/env python3
"""Watch a detached plan and automatically resume unfinished tasks after faults."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from plan_runtime import is_pid_alive, load_json, normalize_manifest_runtime, runtime_state_path, write_json


def classify_watchdog_action(manifest: dict | None, runtime: dict | None, runner_alive: bool) -> tuple[str, str]:
    if not manifest:
        return "wait", "manifest_missing"
    counts = manifest.get("counts") or {}
    pending = int(counts.get("pending", 0))
    running = int(counts.get("running", 0))
    failed = int(counts.get("failed", 0))
    ok = int(counts.get("ok", 0))
    total = len(manifest.get("tasks", []))

    if runner_alive:
        return "wait", "runner_alive"
    if total > 0 and ok == total and pending == 0 and running == 0 and failed == 0:
        return "done", "all_tasks_ok"
    if pending > 0 or running > 0 or failed > 0:
        return "resume", "unfinished_tasks_without_runner"
    return "done", "no_unfinished_tasks"


def launch_detached(repo_root: Path, tool_root: Path, plan_path: Path, *, clean: bool, serve_dashboard: bool, resume: bool) -> dict:
    cmd = [
        sys.executable,
        str(tool_root / "run_simulation_plan.py"),
        "--plan",
        str(plan_path),
        "--detach",
    ]
    if clean:
        cmd.append("--clean")
    if serve_dashboard:
        cmd.append("--serve-dashboard")
    if resume:
        cmd.extend(["--resume", "--resume-include-statuses", "pending,running,failed"])
    proc = subprocess.run(
        cmd,
        cwd=str(repo_root),
        capture_output=True,
        text=True,
        timeout=25.0,
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout or "watchdog launch failed").strip())
    return json.loads(proc.stdout.strip() or "{}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Watch a detached simulation plan and resume unfinished tasks automatically.")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--serve-dashboard", action="store_true")
    ap.add_argument("--clean-start", action="store_true")
    ap.add_argument("--poll-s", type=float, default=20.0)
    ap.add_argument("--max-resumes", type=int, default=8)
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    plan_path = (repo_root / args.plan).resolve() if not Path(args.plan).is_absolute() else Path(args.plan).resolve()

    state_path = tool_root / "results" / f"{plan_path.stem}_watchdog_state.json"
    state = {
        "plan": str(plan_path),
        "started_at": time.time(),
        "resume_count": 0,
        "status": "starting",
        "last_reason": "",
        "last_launch": None,
    }
    write_json(state_path, state)

    launch = launch_detached(
        repo_root,
        tool_root,
        plan_path,
        clean=args.clean_start,
        serve_dashboard=args.serve_dashboard,
        resume=False,
    )
    state["last_launch"] = launch
    state["status"] = "running"
    write_json(state_path, state)

    manifest_path = Path(launch["manifest"]).resolve()
    results_root = Path(launch["results_root"]).resolve()

    while True:
        time.sleep(max(2.0, float(args.poll_s)))
        manifest = load_json(manifest_path)
        runtime = load_json(runtime_state_path(results_root))
        if manifest:
            normalized, changed = normalize_manifest_runtime(manifest)
            if changed:
                manifest = normalized
                write_json(manifest_path, manifest)
        runner_alive = is_pid_alive((runtime or {}).get("pid"), cmdline_contains="run_simulation_plan.py")
        action, reason = classify_watchdog_action(manifest, runtime, runner_alive)
        state["last_reason"] = reason
        state["runner_alive"] = runner_alive
        state["status"] = action
        state["updated_at"] = time.time()
        write_json(state_path, state)

        if action == "wait":
            continue
        if action == "done":
            state["status"] = "finished"
            state["finished_at"] = time.time()
            write_json(state_path, state)
            return 0

        if state["resume_count"] >= int(args.max_resumes):
            state["status"] = "failed"
            state["finished_at"] = time.time()
            write_json(state_path, state)
            raise RuntimeError(f"watchdog exceeded max resumes ({args.max_resumes})")

        launch = launch_detached(
            repo_root,
            tool_root,
            plan_path,
            clean=False,
            serve_dashboard=args.serve_dashboard,
            resume=True,
        )
        state["resume_count"] += 1
        state["last_launch"] = launch
        state["status"] = "running"
        write_json(state_path, state)
        manifest_path = Path(launch["manifest"]).resolve()
        results_root = Path(launch["results_root"]).resolve()


if __name__ == "__main__":
    raise SystemExit(main())
