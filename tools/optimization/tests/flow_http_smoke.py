#!/usr/bin/env python3
"""End-to-end HTTP smoke test for planner -> live monitor -> review flow."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import quote
from pathlib import Path


def wait_for_url(url: str, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while True:
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                if response.status == 200:
                    return True
        except Exception:
            pass
        if time.time() > deadline:
            return False
        time.sleep(0.25)


def request_json(url: str, *, method: str = "GET", payload: dict | None = None, timeout: float = 10.0) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser(description="HTTP smoke test for planner/live/review flow.")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--server-root", default="Tools/optimization")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    server_root = (repo_root / args.server_root).resolve()
    base = f"http://127.0.0.1:{args.port}"

    server_proc = None
    planner_url = f"{base}/planner?fresh={int(time.time())}"
    dashboard_url = f"{base}/dashboard?fresh={int(time.time())}"
    review_url = f"{base}/review?fresh={int(time.time())}"

    try:
        if not wait_for_url(planner_url, 1.0):
            server_proc = subprocess.Popen(
                [sys.executable, str(server_root / "serve_dashboard.py"), "--port", str(args.port), "--root", str(server_root)],
                cwd=str(repo_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            if not wait_for_url(planner_url, 30.0):
                raise RuntimeError("planner server did not become ready")

        request_json(f"{base}/api/stop_plan", method="POST", payload={})

        plan_name = f"flow_smoke_{int(time.time())}"
        plan_path = f"Tools/optimization/generated_plans/{plan_name}.yaml"
        plan = {
            "name": plan_name,
            "rootfs": "build/px4_sitl_default/rootfs",
            "results_root": f"Tools/optimization/plan_runs/{plan_name}",
            "report_html": "Tools/optimization/controller_suite_report.html",
            "defaults": {
                "workers": 1,
                "iterations": 2,
                "global_iters": 1,
                "optimizer": "bayes",
                "takeoff_alt": 2.0,
                "takeoff_timeout": 25.0,
                "trajectory_timeout": "auto",
                "sim_speed_factor": 1.0,
                "simulator": "gz",
                "simulator_vehicle": "x500",
                "simulator_world": "default",
                "max_evals_per_boot": 1,
            },
            "tasks": [
                {
                    "id": "ui_pid_traj0",
                    "controller": "pid",
                    "traj_id": 0,
                }
            ],
        }

        saved = request_json(
            f"{base}/api/save_plan",
            method="POST",
            payload={"filename": plan_path, "plan": plan},
        )
        if not saved.get("ok"):
            raise RuntimeError(f"save failed: {saved}")

        started = request_json(
            f"{base}/api/start_plan",
            method="POST",
            payload={"plan_path": plan_path, "clean": True, "serve_dashboard": False},
            timeout=25.0,
        )
        if not started.get("ok"):
            raise RuntimeError(f"start failed: {started}")

        active = None
        deadline = time.time() + 25.0
        while time.time() < deadline:
            state = request_json(f"{base}/api/dashboard_state", timeout=5.0)
            if state.get("active_plan"):
                active = state
                break
            time.sleep(0.5)
        if not active or not active.get("active_plan"):
            raise RuntimeError("active plan never appeared in dashboard state")

        with urllib.request.urlopen(dashboard_url, timeout=5.0) as response:
            if response.status != 200:
                raise RuntimeError("dashboard did not return 200")
        with urllib.request.urlopen(review_url, timeout=5.0) as response:
            if response.status != 200:
                raise RuntimeError("review did not return 200")

        results_root = str(active["active_plan"]["results_root"])
        history = request_json(
            f"{base}/api/task_history?results_root={quote(results_root)}&task_id=ui_pid_traj0",
            timeout=5.0,
        )
        if not history.get("ok"):
            raise RuntimeError(f"task history request failed: {history}")

        stopped = request_json(f"{base}/api/stop_plan", method="POST", payload={}, timeout=10.0)
        if not stopped.get("ok"):
            raise RuntimeError(f"stop failed: {stopped}")

        idle_state = None
        deadline = time.time() + 20.0
        while time.time() < deadline:
            idle_state = request_json(f"{base}/api/dashboard_state", timeout=5.0)
            if not idle_state.get("active_plan"):
                break
            time.sleep(0.5)
        if idle_state and idle_state.get("active_plan"):
            raise RuntimeError("active plan pointer did not clear after stop")

        print(json.dumps({
            "ok": True,
            "plan_name": plan_name,
            "saved_path": saved.get("path"),
            "active_task": active.get("current_task"),
            "review_ok": True,
            "dashboard_ok": True,
            "stop_message": stopped.get("message"),
        }, indent=2))
        return 0
    finally:
        if server_proc is not None and server_proc.poll() is None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
