#!/usr/bin/env python3
"""Gazebo-backed end-to-end smoke test for planner -> optimizer -> review -> replay."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from urllib.parse import quote


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
    ap = argparse.ArgumentParser(description="Run a real Gazebo-backed e2e smoke plan.")
    ap.add_argument("--port", type=int, default=8094)
    ap.add_argument("--server-root", default="Tools/optimization")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    server_root = (repo_root / args.server_root).resolve()
    base = f"http://127.0.0.1:{args.port}"
    planner_url = f"{base}/planner?fresh={int(time.time())}"

    server_proc = None
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
        request_json(f"{base}/api/stop_replay", method="POST", payload={})

        plan_name = f"gz_e2e_smoke_{int(time.time())}"
        plan_path = f"Tools/optimization/generated_plans/{plan_name}.yaml"
        results_root = repo_root / "Tools" / "optimization" / "plan_runs" / plan_name
        plan = {
            "name": plan_name,
            "rootfs": "build/px4_sitl_default/rootfs",
            "results_root": str(results_root),
            "report_html": "Tools/optimization/controller_suite_report.html",
            "defaults": {
                "workers": 1,
                "iterations": 1,
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
                    "id": "gz_pid_traj0",
                    "controller": "pid",
                    "traj_id": 0,
                }
            ],
        }
        saved = request_json(f"{base}/api/save_plan", method="POST", payload={"filename": plan_path, "plan": plan})
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

        final_state = None
        deadline = time.time() + 420.0
        while time.time() < deadline:
            state = request_json(f"{base}/api/dashboard_state", timeout=5.0)
            active = state.get("active_plan") or {}
            counts = (active.get("counts") or {}) if isinstance(active, dict) else {}
            if counts.get("ok", 0) + counts.get("failed", 0) >= 1 and counts.get("running", 0) == 0:
                final_state = state
                break
            time.sleep(1.0)
        if final_state is None:
            raise RuntimeError("smoke plan did not finish in time")

        counts = final_state["active_plan"]["counts"]
        if counts.get("failed", 0) != 0:
            raise RuntimeError(f"smoke plan failed: {counts}")

        history = request_json(
            f"{base}/api/task_history?results_root={quote(str(results_root))}&task_id=gz_pid_traj0",
            timeout=10.0,
        )
        if not history.get("ok") or not history.get("records"):
            raise RuntimeError(f"task history missing: {history}")

        trace = request_json(
            f"{base}/api/trace_from_eval?results_root={quote(str(results_root))}&task_id=gz_pid_traj0&eval_index=0",
            timeout=10.0,
        )
        if not trace.get("ok") or len(trace.get("t", [])) == 0:
            raise RuntimeError(f"trace missing: {trace}")

        best_path = results_root / "gz_pid_traj0" / "best.json"
        best_payload = json.loads(best_path.read_text(encoding="utf-8"))
        best_params = best_payload.get("full_best_params") or best_payload.get("best_params") or {}
        replay_start = request_json(
            f"{base}/api/start_replay",
            method="POST",
            payload={
                "rootfs": "build/px4_sitl_default/rootfs",
                "build_dir": "build/px4_sitl_default",
                "controller": "pid",
                "traj_id": 0,
                "params": best_params,
                "fixed_params": {},
                "takeoff_alt": 2.0,
                "takeoff_timeout": 25.0,
                "trajectory_timeout": 51.2,
                "w_track": 1.0,
                "w_energy": 0.05,
                "landing_mode": "land",
                "trace_window": "offboard",
                "engagement_dwell_s": 2.0,
                "simulator": "gz",
                "simulator_vehicle": "x500",
                "simulator_world": "default",
                "strict_eval": True,
                "headless": True,
            },
            timeout=20.0,
        )
        if not replay_start.get("ok"):
            raise RuntimeError(f"replay start failed: {replay_start}")

        replay_status = None
        deadline = time.time() + 300.0
        while time.time() < deadline:
            replay_status = request_json(f"{base}/api/replay_status", timeout=5.0)
            if replay_status.get("status") in {"finished", "failed"}:
                break
            time.sleep(1.0)
        if replay_status is None or replay_status.get("status") != "finished":
            raise RuntimeError(f"replay did not finish cleanly: {replay_status}")

        replay_trace = request_json(f"{base}/api/replay_trace", timeout=10.0)
        if not replay_trace.get("ok") or len(replay_trace.get("t", [])) == 0:
            raise RuntimeError(f"replay trace missing: {replay_trace}")

        request_json(f"{base}/api/stop_replay", method="POST", payload={})

        print(json.dumps({
            "ok": True,
            "plan_name": plan_name,
            "results_root": str(results_root),
            "counts": counts,
            "trace_points": len(trace.get("t", [])),
            "replay_points": len(replay_trace.get("t", [])),
            "best_cost": best_payload.get("best_cost"),
        }, indent=2))
        return 0
    finally:
        try:
            request_json(f"{base}/api/stop_replay", method="POST", payload={})
        except Exception:
            pass
        if server_proc is not None and server_proc.poll() is None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
