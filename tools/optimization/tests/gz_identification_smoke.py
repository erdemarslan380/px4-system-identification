#!/usr/bin/env python3
"""Gazebo-backed identification smoke test for the sysid mission pipeline."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from http.client import RemoteDisconnected
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


def request_json_retry(url: str, *, method: str = "GET", payload: dict | None = None, timeout: float = 10.0,
                       attempts: int = 5, delay_s: float = 0.5) -> dict:
    last_error: Exception | None = None
    for _ in range(max(1, attempts)):
        try:
            return request_json(url, method=method, payload=payload, timeout=timeout)
        except (urllib.error.URLError, TimeoutError, ConnectionError, RemoteDisconnected) as exc:
            last_error = exc
            time.sleep(delay_s)
    if last_error is not None:
        raise last_error
    raise RuntimeError("request_json_retry exhausted without a captured error")


def main() -> int:
    ap = argparse.ArgumentParser(description="Run a real Gazebo-backed identification smoke plan.")
    ap.add_argument("--port", type=int, default=8095)
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

        for endpoint in ("/api/stop_plan", "/api/stop_replay"):
            try:
                request_json_retry(f"{base}{endpoint}", method="POST", payload={}, timeout=10.0)
            except Exception:
                # Best-effort cleanup. A fresh port may legitimately have no active runtime yet.
                pass

        plan_name = f"gz_ident_smoke_{int(time.time())}"
        plan_path = f"Tools/optimization/generated_plans/{plan_name}.yaml"
        results_root = repo_root / "Tools" / "optimization" / "plan_runs" / plan_name
        out_dir = repo_root / "experimental_validation" / "outputs" / plan_name

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
                "trajectory_timeout": 45.0,
                "sim_speed_factor": 1.0,
                "simulator": "gz",
                "simulator_vehicle": "x500",
                "simulator_world": "default",
                "max_evals_per_boot": 1,
                "mission_mode": "identification",
                "ident_profile": "hover_thrust",
            },
            "tasks": [
                {
                    "id": "gz_sysid_hover",
                    "controller": "sysid",
                    "traj_id": 0,
                    "mission_mode": "identification",
                    "ident_profile": "hover_thrust",
                }
            ],
        }

        saved = request_json_retry(f"{base}/api/save_plan", method="POST", payload={"filename": plan_path, "plan": plan})
        if not saved.get("ok"):
            raise RuntimeError(f"save failed: {saved}")

        started = request_json_retry(
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
            state = request_json_retry(f"{base}/api/dashboard_state", timeout=5.0)
            active = state.get("active_plan") or {}
            counts = (active.get("counts") or {}) if isinstance(active, dict) else {}
            if counts.get("ok", 0) + counts.get("failed", 0) >= 1 and counts.get("running", 0) == 0:
                final_state = state
                break
            time.sleep(1.0)
        if final_state is None:
            raise RuntimeError("identification smoke plan did not finish in time")

        counts = final_state["active_plan"]["counts"]
        if counts.get("failed", 0) != 0:
            raise RuntimeError(f"identification smoke failed: {counts}")

        task_dir = results_root / "gz_sysid_hover"
        eval_payload = None
        for eval_path in sorted(task_dir.glob("eval_*.json")):
            candidate = json.loads(eval_path.read_text(encoding="utf-8"))
            if not candidate.get("failed"):
                eval_payload = candidate
                break
        if eval_payload is None:
            raise RuntimeError("identification smoke did not produce a successful evaluation")
        ident_log = Path(
            str(
                eval_payload.get("identification_log_archive")
                or eval_payload.get("identification_log")
                or ""
            )
        ).resolve()
        if not ident_log.exists():
            raise RuntimeError(f"identification log missing: {ident_log}")
        truth_log = Path(
            str(
                eval_payload.get("gazebo_truth_log_archive")
                or ""
            )
        ).resolve()
        if not truth_log.exists():
            raise RuntimeError(f"gazebo truth log missing: {truth_log}")

        cli_proc = subprocess.run(
            [
                sys.executable,
                str(repo_root / "experimental_validation" / "cli.py"),
                "--csv", str(ident_log),
                "--truth-csv", str(truth_log),
                "--ident-log",
                "--out-dir", str(out_dir),
            ],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=60.0,
        )
        if cli_proc.returncode != 0:
            raise RuntimeError(f"experimental_validation cli failed:\n{cli_proc.stdout}\n{cli_proc.stderr}")

        identified_json = out_dir / "identified_parameters.json"
        inertial_xml = out_dir / "candidate_inertial.sdf.xml"
        vehicle_yaml = out_dir / "candidate_vehicle_params.yaml"
        if not identified_json.exists() or not inertial_xml.exists() or not vehicle_yaml.exists():
            raise RuntimeError("experimental validation outputs are incomplete")

        compare_proc = subprocess.run(
            [
                sys.executable,
                str(repo_root / "experimental_validation" / "compare_with_sdf.py"),
                "--results-root", str(results_root),
                "--out-dir", str(out_dir),
            ],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=60.0,
        )
        if compare_proc.returncode != 0:
            raise RuntimeError(f"sdf comparison failed:\n{compare_proc.stdout}\n{compare_proc.stderr}")
        comparison_payload = json.loads((out_dir / "sdf_comparison.json").read_text(encoding="utf-8"))
        comparable = comparison_payload["comparable_metrics"]
        for metric_name in (
            "mass_kg",
            "ixx_kgm2",
            "iyy_kgm2",
            "izz_kgm2",
            "motor_constant",
            "moment_constant",
            "rotor_drag_coefficient",
            "rolling_moment_coefficient",
            "rotor_velocity_slowdown_sim",
        ):
            metric = comparable[metric_name]
            if abs(float(metric["pct_error"])) > 5.0:
                raise RuntimeError(f"{metric_name} error too high: {metric}")

        eval_index = int(eval_payload.get("eval_id") or 0)
        trace = request_json_retry(
            f"{base}/api/trace_from_eval?results_root={quote(str(results_root))}&task_id=gz_sysid_hover&eval_index={eval_index}",
            timeout=10.0,
        )
        if not trace.get("ok") or len(trace.get("t", [])) == 0:
            raise RuntimeError(f"trace missing: {trace}")

        replay_start = request_json(
            f"{base}/api/start_replay",
            method="POST",
            payload={
                "rootfs": "build/px4_sitl_default/rootfs",
                "build_dir": "build/px4_sitl_default",
                "controller": "sysid",
                "traj_id": 0,
                "params": {},
                "fixed_params": {},
                "takeoff_alt": 2.0,
                "takeoff_timeout": 25.0,
                "trajectory_timeout": 45.0,
                "w_track": 1.0,
                "w_energy": 0.05,
                "landing_mode": "land",
                "trace_window": "offboard",
                "engagement_dwell_s": 2.0,
                "mission_mode": "identification",
                "ident_profile": "hover_thrust",
                "simulator": "gz",
                "simulator_vehicle": "x500",
                "simulator_world": "default",
                "strict_eval": True,
                "headless": False,
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
        if replay_status.get("display_mode") != "gui":
            raise RuntimeError(f"replay did not use GUI mode: {replay_status}")

        replay_trace = request_json(f"{base}/api/replay_trace", timeout=10.0)
        if not replay_trace.get("ok") or len(replay_trace.get("t", [])) == 0:
            raise RuntimeError(f"replay trace missing: {replay_trace}")

        request_json(f"{base}/api/stop_replay", method="POST", payload={})

        print(json.dumps({
            "ok": True,
            "plan_name": plan_name,
            "results_root": str(results_root),
            "eval_index": eval_index,
            "identification_log": str(ident_log),
            "gazebo_truth_log": str(truth_log),
            "identified_json": str(identified_json),
            "comparison_json": str(out_dir / "sdf_comparison.json"),
            "trace_points": len(trace.get("t", [])),
            "replay_points": len(replay_trace.get("t", [])),
            "replay_display_mode": replay_status.get("display_mode"),
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
