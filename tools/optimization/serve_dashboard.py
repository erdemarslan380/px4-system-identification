#!/usr/bin/env python3
"""Serve the unified tuning dashboard and its JSON APIs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

from generate_controller_suite_report import collect_cases
from plan_runtime import is_pid_alive, load_json as runtime_load_json, normalize_manifest_runtime, runtime_state_path
from runtime_cleanup import _kill_pid_or_group, _ps_rows, _wait_gone, cleanup_runtime_slots
from ui_catalog import catalog_payload, load_plan_yaml, save_plan_yaml
from trajectory_utils import trajectory_points


class DashboardHandler(SimpleHTTPRequestHandler):
    root_dir = Path(".")
    replay_lock = threading.Lock()
    replay_proc: subprocess.Popen | None = None

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        super().end_headers()

    def _normalize_path(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            suffix = f"?{parsed.query}" if parsed.query else ""
            self.path = f"/dashboard.html{suffix}"
        elif parsed.path == "/live":
            self.path = "/dashboard.html?view=live"
        elif parsed.path == "/review":
            self.path = "/dashboard.html?view=review"
        elif parsed.path == "/planner":
            suffix = f"?{parsed.query}" if parsed.query else ""
            self.path = f"/plan_builder.html{suffix}"

    def _send_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(self._sanitize_json(payload), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _sanitize_json(self, value):
        if isinstance(value, dict):
            return {str(k): self._sanitize_json(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._sanitize_json(v) for v in value]
        if isinstance(value, tuple):
            return [self._sanitize_json(v) for v in value]
        if isinstance(value, float):
            if math.isnan(value) or math.isinf(value):
                return None
            return value
        return value

    def _read_json(self, path: Path) -> dict | None:
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None

    def _file_mtime(self, path: Path) -> float | None:
        try:
            return path.stat().st_mtime
        except Exception:
            return None

    def _load_history(self, results_dir: Path) -> list[dict]:
        path = results_dir / "history.jsonl"
        rows: list[dict] = []
        if not path.exists():
            return rows
        for line in path.read_text(encoding="utf-8").splitlines():
            raw = line.strip()
            if not raw:
                continue
            try:
                rows.append(json.loads(raw))
            except Exception:
                continue
        return rows

    def _active_pointer(self) -> dict | None:
        return self._read_json(self.root_dir / "active_plan.json")

    def _active_manifest(self) -> tuple[dict | None, Path | None]:
        pointer = self._active_pointer()
        if not pointer:
            return None, None
        manifest_path = Path(str(pointer.get("manifest_path", ""))).resolve()
        if not manifest_path.exists():
            return None, None
        manifest = self._read_json(manifest_path)
        if manifest:
            manifest, _ = normalize_manifest_runtime(manifest)
        return manifest, manifest_path

    def _manifest_for_results_root(self, results_root: Path) -> tuple[dict | None, Path | None]:
        manifest_path = results_root / "plan_manifest.json"
        if not manifest_path.exists():
            return None, None
        manifest = self._read_json(manifest_path)
        if manifest:
            manifest, _ = normalize_manifest_runtime(manifest)
        return manifest, manifest_path

    def _discover_manifests(self) -> list[tuple[dict, Path]]:
        out: list[tuple[dict, Path]] = []
        plan_runs = self.root_dir / "plan_runs"
        if not plan_runs.exists():
            return out
        for manifest_path in sorted(plan_runs.glob("*/plan_manifest.json")):
            manifest = self._read_json(manifest_path)
            if not manifest:
                continue
            manifest, _ = normalize_manifest_runtime(manifest)
            out.append((manifest, manifest_path))
        return out

    def _selected_manifest(self, results_root_raw: str | None) -> tuple[dict | None, Path | None]:
        if results_root_raw:
            try:
                results_root = Path(results_root_raw).resolve()
            except Exception:
                return None, None
            return self._manifest_for_results_root(results_root)
        return self._active_manifest()

    def _available_plans(self) -> list[dict]:
        out: list[dict] = []
        active_pointer = self._active_pointer() or {}
        active_results_root = str(active_pointer.get("results_root") or "")
        for manifest, manifest_path in self._discover_manifests():
            results_root = str(Path(manifest["results_root"]).resolve())
            out.append({
                "name": str(manifest.get("name") or Path(results_root).name),
                "results_root": results_root,
                "manifest_path": str(manifest_path),
                "plan_file": str(manifest.get("plan_file") or ""),
                "counts": manifest.get("counts") or manifest.get("summary") or {},
                "started_at": manifest.get("started_at"),
                "finished_at": manifest.get("finished_at"),
                "current_task_id": manifest.get("current_task_id"),
                "is_active": results_root == active_results_root,
            })
        out.sort(key=lambda item: float(item.get("finished_at") or item.get("started_at") or 0.0), reverse=True)
        return out

    def _plan_runtime_state(self, manifest: dict | None) -> tuple[dict, bool]:
        if not manifest:
            return {"status": "idle"}, False

        runtime_path = runtime_state_path(Path(manifest["results_root"]).resolve())
        runtime_state = runtime_load_json(runtime_path) or {"status": "idle"}
        pid = runtime_state.get("pid")
        runtime_live = (
            str(runtime_state.get("status", "")) == "running"
            and is_pid_alive(pid, cmdline_contains="run_simulation_plan.py")
        )
        if runtime_live:
            return runtime_state, True

        if str(runtime_state.get("status", "")) == "running":
            runtime_state = {
                **runtime_state,
                "status": "stale",
                "error": "plan runtime pid missing or mismatched",
            }
        return runtime_state, False

    def _results_dir_for_task(self, manifest: dict | None, task_id: str | None) -> tuple[Path, dict | None]:
        if manifest and task_id:
            for task in manifest.get("tasks", []):
                if str(task.get("task_id")) == task_id:
                    results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / task_id))).resolve()
                    return results_dir, task
        if manifest:
            current_id = manifest.get("current_task_id")
            if current_id:
                for task in manifest.get("tasks", []):
                    if str(task.get("task_id")) == str(current_id):
                        results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / current_id))).resolve()
                        return results_dir, task
            finished = [task for task in manifest.get("tasks", []) if task.get("status") in ("ok", "failed")]
            if finished:
                task = finished[-1]
                task_id = str(task.get("task_id"))
                results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / task_id))).resolve()
                return results_dir, task
            pending = manifest.get("tasks", [])
            if pending:
                task = pending[0]
                task_id = str(task.get("task_id"))
                results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / task_id))).resolve()
                return results_dir, task
        return (self.root_dir / "results").resolve(), None

    def _clean_case_payload(self, case: dict) -> dict:
        payload = dict(case)
        payload.pop("start_trace", None)
        payload.pop("best_trace", None)
        payload["start_params"] = {k: float(v) for k, v in dict(payload.get("start_params", {})).items()}
        payload["best_params"] = {k: float(v) for k, v in dict(payload.get("best_params", {})).items()}
        payload["fixed_params"] = {k: float(v) for k, v in dict(payload.get("fixed_params", {})).items()}
        return payload

    def _collect_cases_for_manifest(self, manifest: dict | None) -> list[dict]:
        if manifest:
            results_root = Path(manifest["results_root"]).resolve()
        else:
            results_root = (self.root_dir / "results").resolve()
        cases = collect_cases(results_root, manifest=manifest)
        plan_name = str(manifest.get("name") or results_root.name) if manifest else "legacy"
        for case in cases:
            case["plan_name"] = plan_name
            case["results_root"] = str(results_root)
        return cases

    def _case_sort_time(self, case: dict) -> float:
        for key in ("finished_at", "started_at"):
            raw = case.get(key)
            if isinstance(raw, (int, float)):
                return float(raw)
        return 0.0

    def _prefer_combined_case(self, current: dict, candidate: dict) -> bool:
        cur_status = str(current.get("status", "pending"))
        new_status = str(candidate.get("status", "pending"))
        if new_status == "running" and cur_status != "running":
            return True
        if cur_status == "running" and new_status != "running":
            return False
        if cur_status == "ok" and new_status in {"pending", "failed"}:
            return False
        if new_status == "ok" and cur_status in {"pending", "failed"}:
            return True
        return self._case_sort_time(candidate) >= self._case_sort_time(current)

    def _combined_report_summary(self, active_manifest: dict | None) -> dict:
        merged: dict[tuple[str, int, str], dict] = {}
        for manifest, _path in self._discover_manifests():
            for case in self._collect_cases_for_manifest(manifest):
                key = (
                    str(case.get("controller", "")),
                    int(case.get("traj_id", -1)),
                    str(case.get("optimizer", "bayes")),
                )
                current = merged.get(key)
                if current is None or self._prefer_combined_case(current, case):
                    merged[key] = case
        clean_cases = [self._clean_case_payload(case) for case in merged.values()]
        clean_cases.sort(key=lambda case: (
            int(case.get("traj_id", -1)),
            str(case.get("optimizer", "bayes")),
            str(case.get("controller", "")),
        ))
        return {
            "cases": clean_cases,
            "summary": {
                "total": len(clean_cases),
                "ok": sum(1 for case in clean_cases if case.get("status") == "ok"),
                "running": sum(1 for case in clean_cases if case.get("status") == "running"),
                "pending": sum(1 for case in clean_cases if case.get("status") == "pending"),
                "failed": sum(1 for case in clean_cases if case.get("status") == "failed"),
            },
            "scope": "combined_latest",
        }

    def _report_summary(self, manifest: dict | None) -> dict:
        cases = self._collect_cases_for_manifest(manifest)
        clean_cases = [self._clean_case_payload(case) for case in cases]
        return {
            "cases": clean_cases,
            "summary": {
                "total": len(clean_cases),
                "ok": sum(1 for case in clean_cases if case.get("status") == "ok"),
                "running": sum(1 for case in clean_cases if case.get("status") == "running"),
                "pending": sum(1 for case in clean_cases if case.get("status") == "pending"),
                "failed": sum(1 for case in clean_cases if case.get("status") == "failed"),
            },
            "scope": "active_plan",
        }

    def _results_dir_from_request(self, results_dir_raw: str | None, manifest: dict | None, task_id: str | None) -> tuple[Path, dict | None]:
        if results_dir_raw:
            results_dir = Path(results_dir_raw).resolve()
            if results_dir.exists() and results_dir.is_dir():
                task = None
                task_manifest, _ = self._manifest_for_results_root(results_dir.parent)
                for source_manifest in (task_manifest, manifest):
                    if source_manifest and task_id:
                        for candidate in source_manifest.get("tasks", []):
                            if str(candidate.get("task_id")) == str(task_id):
                                task = candidate
                                break
                    if task is not None:
                        break
                return results_dir, task
        return self._results_dir_for_task(manifest, task_id)

    def _replay_paths(self) -> dict[str, Path]:
        results_dir = self.root_dir / "results"
        results_dir.mkdir(parents=True, exist_ok=True)
        return {
            "state_json": results_dir / "replay_state.json",
            "result_json": results_dir / "replay_result.json",
            "live_trace_json": results_dir / "replay_live_trace.json",
            "log_path": results_dir / "replay_launcher.log",
        }

    def _history_stats(self, rows: list[dict]) -> dict:
        success = [row for row in rows if not row.get("failed", False)]
        return {
            "total": len(rows),
            "successful": len(success),
            "failed": len(rows) - len(success),
            "latest_eval_index": int(rows[-1]["eval_index"]) if rows else None,
            "latest_success_eval_index": int(success[-1]["eval_index"]) if success else None,
        }

    def _estimate_plan_progress(self, manifest: dict | None, current_task_id: str | None, current_pool_status: dict | None) -> dict | None:
        if not manifest:
            return None

        tasks = manifest.get("tasks", [])
        total_expected = 0
        total_completed = 0
        completed_tasks = 0
        remaining_eta_s = 0.0
        controller_wall_per_eval: dict[str, list[float]] = {}
        global_wall_per_eval: list[float] = []
        task_summaries = []

        def finished_task_estimate(task: dict, expected: int, results_dir: Path) -> tuple[int, float | None]:
            rows = self._load_history(results_dir)
            actual_completed = len(rows)
            duration_s = task.get("duration_s")
            wall_per_eval = None
            if isinstance(duration_s, (int, float)) and duration_s > 0:
                denom = max(1, actual_completed if actual_completed > 0 else expected)
                wall_per_eval = float(duration_s) / denom
            return actual_completed, wall_per_eval

        for task in tasks:
            task_id = str(task.get("task_id", ""))
            controller = str(task.get("controller", ""))
            results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / task_id))).resolve()
            expected = int(task.get("total_expected_evals") or 0)
            if expected <= 0:
                run_cfg = self._read_json(results_dir / "run_config.json") or {}
                expected = int(run_cfg.get("total_expected_evals") or 0)
            total_expected += expected

            status = str(task.get("status") or "pending")
            actual_completed = 0

            if status in ("ok", "failed"):
                actual_completed, wall_per_eval = finished_task_estimate(task, expected, results_dir)
                total_completed += actual_completed
                if status == "ok":
                    completed_tasks += 1
                if wall_per_eval is not None:
                    controller_wall_per_eval.setdefault(controller, []).append(wall_per_eval)
                    global_wall_per_eval.append(wall_per_eval)
                task_summaries.append({"task_id": task_id, "status": status, "completed": actual_completed, "expected": expected})
                continue

            if status == "running" and task_id == str(current_task_id or ""):
                rows = self._load_history(results_dir)
                actual_completed = int((current_pool_status or {}).get("completed_evals") or len(rows))
                total_completed += actual_completed
                eta_s = (current_pool_status or {}).get("eta_s")
                if isinstance(eta_s, (int, float)) and eta_s >= 0:
                    remaining_eta_s += float(eta_s)
                task_summaries.append({"task_id": task_id, "status": status, "completed": actual_completed, "expected": expected})
                continue

            task_summaries.append({"task_id": task_id, "status": status, "completed": 0, "expected": expected})

        fallback_wall_per_eval = sum(global_wall_per_eval) / len(global_wall_per_eval) if global_wall_per_eval else None

        for task in tasks:
            task_id = str(task.get("task_id", ""))
            status = str(task.get("status") or "pending")
            if status != "pending":
                continue
            expected = int(task.get("total_expected_evals") or 0)
            controller = str(task.get("controller", ""))
            controller_estimates = controller_wall_per_eval.get(controller, [])
            wall_per_eval = (
                sum(controller_estimates) / len(controller_estimates)
                if controller_estimates
                else fallback_wall_per_eval
            )
            if wall_per_eval is not None and expected > 0:
                remaining_eta_s += expected * wall_per_eval

        remaining_tasks = sum(1 for task in tasks if str(task.get("status")) in ("pending", "running"))
        return {
            "tasks_total": len(tasks),
            "tasks_completed": completed_tasks,
            "tasks_remaining": remaining_tasks,
            "evals_total_expected": total_expected,
            "evals_completed": total_completed,
            "remaining_eta_s": remaining_eta_s if remaining_eta_s > 0 else None,
            "task_summaries": task_summaries,
        }

    def _dashboard_state(self, results_root_raw: str | None = None) -> dict:
        manifest, manifest_path = self._selected_manifest(results_root_raw)
        results_dir, task = self._results_dir_for_task(manifest, None)
        run_config = self._read_json(results_dir / "run_config.json") or {}
        pool_status = self._read_json(results_dir / "pool_status.json") or {}
        history_rows = self._load_history(results_dir)
        report = self._report_summary(manifest)
        report_combined = self._combined_report_summary(manifest)
        replay_state = self._read_json(self.root_dir / "results" / "replay_state.json") or {
            "status": "idle",
        }
        runtime_state, runtime_live = self._plan_runtime_state(manifest)
        live_task = task if (runtime_live and task and str(task.get("status", "")) == "running") else None
        if not live_task:
            pool_status = {}
        plan_progress = self._estimate_plan_progress(
            manifest,
            live_task.get("task_id") if live_task else None,
            pool_status,
        )
        active_plan = None
        if manifest:
            active_plan = {
                "name": manifest.get("name"),
                "plan_file": manifest.get("plan_file"),
                "rootfs": manifest.get("rootfs"),
                "results_root": manifest.get("results_root"),
                "report_html": manifest.get("report_html"),
                "started_at": manifest.get("started_at"),
                "finished_at": manifest.get("finished_at"),
                "current_task_id": manifest.get("current_task_id"),
                "current_task_index": manifest.get("current_task_index"),
                "counts": manifest.get("counts", {}),
                "tasks": [
                    {
                        "task_id": task.get("task_id"),
                        "label": task.get("label"),
                        "controller": task.get("controller"),
                        "task_group_id": task.get("task_group_id"),
                        "optimizer": task.get("optimizer", "bayes"),
                        "optimizer_display_name": task.get("optimizer_display_name"),
                        "optimizer_short_label": task.get("optimizer_short_label"),
                        "optimizer_color": task.get("optimizer_color"),
                        "traj_id": task.get("traj_id"),
                        "status": task.get("status"),
                        "status_detail": task.get("status_detail"),
                        "iterations": task.get("iterations"),
                        "global_iters": task.get("global_iters"),
                        "local_iters": task.get("local_iters"),
                        "param_dim": task.get("param_dim"),
                        "total_expected_evals": task.get("total_expected_evals"),
                        "trajectory_duration_s": task.get("trajectory_duration_s"),
                        "trajectory_timeout": task.get("trajectory_timeout"),
                        "started_at": task.get("started_at"),
                        "finished_at": task.get("finished_at"),
                        "duration_s": task.get("duration_s"),
                        "successful_evals": task.get("successful_evals"),
                        "workers_effective": task.get("workers_effective"),
                    }
                    for task in manifest.get("tasks", [])
                ],
            }
        return {
            "ok": True,
            "mode": "plan" if manifest else "single",
            "selected_results_root": str(Path(manifest["results_root"]).resolve()) if manifest else "",
            "manifest_path": str(manifest_path) if manifest_path else "",
            "active_plan": active_plan,
            "current_task": live_task,
            "current_results_dir": str(results_dir),
            "current_run_config": run_config,
            "current_pool_status": pool_status,
            "current_pool_status_mtime": self._file_mtime(results_dir / "pool_status.json"),
            "current_history_stats": self._history_stats(history_rows),
            "current_history_mtime": self._file_mtime(results_dir / "history.jsonl"),
            "report": report,
            "report_combined": report_combined,
            "plan_runtime": runtime_state,
            "plan_progress": plan_progress,
            "replay_state": replay_state,
            "available_plans": self._available_plans(),
            "time": time.time(),
        }

    def _kill_replay_runtime(self) -> None:
        payload = self._read_json(self._replay_paths()["state_json"]) or {}
        for key in ("px4_pid", "sim_pid", "jmavsim_pid", "gui_pid"):
            raw = payload.get(key)
            try:
                pid = int(raw)
            except Exception:
                continue
            if pid <= 0:
                continue
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(pid, sig)
                except Exception:
                    try:
                        os.kill(pid, sig)
                    except Exception:
                        pass
                time.sleep(0.1)
        try:
            cleanup_runtime_slots(
                self.root_dir.parents[1],
                instance_ids=[40],
                ports=[4600],
                process_hints=[
                    str((self.root_dir.parents[1] / "build" / "px4_sitl_default" / "visual_replay_040").resolve()),
                ],
            )
        except Exception:
            pass

    def _trace_from_eval(self, results_dir: Path, eval_index: int) -> dict:
        eval_json = results_dir / f"eval_{eval_index:05d}.json"
        if not eval_json.exists():
            return {
                "ok": False,
                "message": f"eval json not found: {eval_json.name}",
                "eval_index": eval_index,
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }

        try:
            meta = json.loads(eval_json.read_text(encoding="utf-8"))
        except Exception as exc:
            return {
                "ok": False,
                "message": f"eval json parse error: {exc}",
                "eval_index": eval_index,
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }

        log_raw = str(meta.get("tracking_log", "")).strip()
        archive_raw = str(meta.get("tracking_log_archive", "")).strip()
        log_path = Path(log_raw) if log_raw else Path("__missing__")
        archive_path = Path(archive_raw) if archive_raw else None
        if ((not log_path.exists()) or log_path.is_dir()) and archive_path is not None and archive_path.exists() and archive_path.is_file():
            log_path = archive_path

        if not log_path.exists() or log_path.is_dir():
            missing_hint = archive_path if archive_path is not None else log_path
            return {
                "ok": False,
                "message": f"tracking log missing: {missing_hint}",
                "eval_index": eval_index,
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }

        payload = {
            "ok": True,
            "message": "ok",
            "eval_index": eval_index,
            "fitness": meta.get("fitness"),
            "track_rmse": meta.get("track_rmse"),
            "energy_term": meta.get("energy_term"),
            "tracking_log": str(log_path),
            "t": [],
            "ref": {"x": [], "y": [], "z": []},
            "act": {"x": [], "y": [], "z": []},
        }

        try:
            t0_us = None
            with log_path.open("r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ts_us = int(float(row["timestamp_us"]))
                    if t0_us is None:
                        t0_us = ts_us
                    payload["t"].append((ts_us - t0_us) * 1e-6)
                    payload["ref"]["x"].append(float(row["ref_x"]))
                    payload["ref"]["y"].append(float(row["ref_y"]))
                    payload["ref"]["z"].append(float(row["ref_z"]))
                    payload["act"]["x"].append(float(row["pos_x"]))
                    payload["act"]["y"].append(float(row["pos_y"]))
                    payload["act"]["z"].append(float(row["pos_z"]))
        except Exception as exc:
            return {
                "ok": False,
                "message": f"tracking log parse error: {exc}",
                "eval_index": eval_index,
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }

        return payload

    def _replay_trace(self) -> dict:
        paths = self._replay_paths()
        trace_path = paths["live_trace_json"]
        if not trace_path.exists():
            return {
                "ok": False,
                "message": "replay trace missing",
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }
        try:
            payload = json.loads(trace_path.read_text(encoding="utf-8"))
        except Exception as exc:
            return {
                "ok": False,
                "message": f"replay trace parse error: {exc}",
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }
        payload["ok"] = True
        return payload

    def _live_trace(self, results_dir: Path, worker_id: int | None = None) -> dict:
        pool_status = self._read_json(results_dir / "pool_status.json") or {}
        chosen = worker_id
        if chosen is None:
            chosen = 0
        trace_path = results_dir / "live_traces" / f"worker_{int(chosen):02d}.json"
        if not trace_path.exists():
            fallback = pool_status.get("active_worker_id")
            if fallback is not None and int(fallback) != int(chosen):
                alt_path = results_dir / "live_traces" / f"worker_{int(fallback):02d}.json"
                if alt_path.exists():
                    chosen = int(fallback)
                    trace_path = alt_path
        if not trace_path.exists():
            return {
                "ok": False,
                "message": f"live trace missing: {trace_path.name}",
                "worker_id": int(chosen),
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }
        try:
            payload = json.loads(trace_path.read_text(encoding="utf-8"))
        except Exception as exc:
            return {
                "ok": False,
                "message": f"live trace parse error: {exc}",
                "worker_id": int(chosen),
                "t": [],
                "ref": {"x": [], "y": [], "z": []},
                "act": {"x": [], "y": [], "z": []},
            }
        payload["ok"] = True
        payload["worker_id"] = int(chosen)
        return payload

    def _terminate_replay_locked(self) -> None:
        proc = DashboardHandler.replay_proc
        if proc is None or proc.poll() is not None:
            DashboardHandler.replay_proc = None
            self._kill_replay_runtime()
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=8.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3.0)
            except Exception:
                pass
        self._kill_replay_runtime()
        DashboardHandler.replay_proc = None

    def _idle_replay_capacity(self) -> tuple[bool, str]:
        manifest, _ = self._active_manifest()
        if not manifest:
            return True, "isolated replay instance"
        runtime = runtime_load_json(runtime_state_path(Path(manifest["results_root"]).resolve())) or {}
        if runtime.get("status") != "running" or not is_pid_alive(runtime.get("pid"), cmdline_contains="run_simulation_plan.py"):
            return True, "isolated replay instance"
        current_id = manifest.get("current_task_id")
        if not current_id:
            return True, "isolated replay instance"
        results_dir, _ = self._results_dir_for_task(manifest, str(current_id))
        pool_status = self._read_json(results_dir / "pool_status.json") or {}
        idle = int(pool_status.get("workers_idle", 0))
        if idle < 1:
            return True, "isolated replay instance outside worker pool"
        return True, f"idle_capacity={idle} + isolated replay instance"

    def _start_replay(self, body: dict) -> dict:
        allowed, msg = self._idle_replay_capacity()
        if not allowed:
            return {
                "ok": False,
                "message": msg,
            }

        replay_paths = self._replay_paths()
        state_json = replay_paths["state_json"]
        result_json = replay_paths["result_json"]
        live_trace_json = replay_paths["live_trace_json"]
        log_path = replay_paths["log_path"]

        cmd = [
            sys.executable,
            str(self.root_dir / "visual_replay.py"),
            "--rootfs", str(body.get("rootfs") or "build/px4_sitl_default/rootfs"),
            "--controller", str(body["controller"]),
            "--traj-id", str(int(body["traj_id"])),
            "--params-json", json.dumps(body.get("params", {})),
            "--fixed-params-json", json.dumps(body.get("fixed_params", {})),
            "--state-json", str(state_json),
            "--result-json", str(result_json),
            "--live-trace-json", str(live_trace_json),
            "--takeoff-alt", str(float(body.get("takeoff_alt", 2.0))),
            "--takeoff-timeout", str(float(body.get("takeoff_timeout", 40.0))),
            "--trajectory-timeout", str(float(body.get("trajectory_timeout", 180.0))),
            "--w-track", str(float(body.get("w_track", 1.0))),
            "--w-energy", str(float(body.get("w_energy", 0.05))),
            "--landing-mode", str(body.get("landing_mode") or "land"),
            "--trace-window", str(body.get("trace_window") or "offboard"),
            "--engagement-dwell-s", str(float(body.get("engagement_dwell_s", 2.0))),
            "--mission-mode", str(body.get("mission_mode") or "trajectory"),
            "--ident-profile", str(body.get("ident_profile") or "hover_thrust"),
            "--simulator", str(body.get("simulator") or "gz"),
            "--simulator-vehicle", str(body.get("simulator_vehicle") or "x500"),
            "--simulator-world", str(body.get("simulator_world") or "default"),
        ]
        build_dir = str(body.get("build_dir") or "").strip()
        if build_dir:
            cmd.extend(["--build-dir", build_dir])
        base_param_file = str(body.get("base_param_file") or "").strip()
        if base_param_file:
            cmd.extend(["--base-param-file", base_param_file])
        if bool(body.get("strict_eval", True)):
            cmd.append("--strict-eval")
        if bool(body.get("headless", False)):
            cmd.append("--headless")

        with self.replay_lock:
            self._terminate_replay_locked()
            try:
                cleanup_runtime_slots(
                    self.root_dir.parents[1],
                    instance_ids=[40],
                    ports=[4600],
                    process_hints=[
                        str((self.root_dir.parents[1] / "build" / "px4_sitl_default" / "visual_replay_040").resolve()),
                    ],
                )
            except Exception:
                pass
            for artifact in (result_json, live_trace_json):
                try:
                    artifact.unlink()
                except FileNotFoundError:
                    pass
            write_payload = {
                "status": "queued",
                "controller": body["controller"],
                "traj_id": int(body["traj_id"]),
                "params": body.get("params", {}),
                "fixed_params": body.get("fixed_params", {}),
                "capacity_note": msg,
                "requested_at": time.time(),
                "mission_mode": str(body.get("mission_mode") or "trajectory"),
                "ident_profile": str(body.get("ident_profile") or "hover_thrust"),
                "base_param_file": base_param_file,
            }
            state_json.write_text(json.dumps(write_payload, indent=2), encoding="utf-8")
            DashboardHandler.replay_proc = subprocess.Popen(
                cmd,
                cwd=str(self.root_dir.parents[1]),
                stdout=log_path.open("w", encoding="utf-8"),
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        return {
            "ok": True,
            "message": "replay started",
            "pid": DashboardHandler.replay_proc.pid if DashboardHandler.replay_proc else None,
        }

    def _stop_replay(self) -> dict:
        paths = self._replay_paths()
        state = self._read_json(paths["state_json"]) or {"status": "idle"}
        with self.replay_lock:
            self._terminate_replay_locked()
        stopped = {
            **state,
            "status": "stopped",
            "stopped_at": time.time(),
        }
        paths["state_json"].write_text(json.dumps(stopped, indent=2), encoding="utf-8")
        return {"ok": True, "message": "replay stopped"}

    def _replay_status(self) -> dict:
        payload = self._read_json(self.root_dir / "results" / "replay_state.json") or {"status": "idle"}
        proc = DashboardHandler.replay_proc
        if proc is not None and proc.poll() is not None and payload.get("status") in ("queued", "starting", "running"):
            payload = {
                **payload,
                "status": "failed",
                "error": f"replay launcher exited with returncode {proc.returncode}",
            }
        if payload.get("status") in ("queued", "starting", "running"):
            px4_pid = int(payload.get("px4_pid", -1)) if str(payload.get("px4_pid", "")).strip() else -1
            sim_pid = int(payload.get("sim_pid", -1)) if str(payload.get("sim_pid", "")).strip() else -1
            if sim_pid <= 0:
                sim_pid = int(payload.get("jmavsim_pid", -1)) if str(payload.get("jmavsim_pid", "")).strip() else -1
            px4_alive = px4_pid > 0 and Path(f"/proc/{px4_pid}").exists()
            sim_alive = sim_pid > 0 and Path(f"/proc/{sim_pid}").exists()
            if (px4_pid > 0 or sim_pid > 0) and not px4_alive and not sim_alive:
                payload = {
                    **payload,
                    "status": "failed",
                    "error": "replay runtime exited unexpectedly",
                }
        return payload

    def _kill_related_plan_processes(self, results_root: Path | None, manifest: dict | None) -> int:
        if not results_root:
            return 0
        results_root = results_root.resolve()
        repo_root = self.root_dir.parents[1]
        matches: set[int] = set()
        hints = [
            str(results_root),
            str(self.root_dir / "run_simulation_plan.py"),
            str(self.root_dir / "orchestrate_parallel.py"),
            str(self.root_dir / "px4_eval.py"),
        ]
        for pid, cmd in _ps_rows():
            if pid == os.getpid():
                continue
            if str(results_root) in cmd:
                matches.add(pid)
                continue
            if any(hint in cmd for hint in hints[:3]) and str(results_root) in cmd:
                matches.add(pid)
        if not matches:
            return 0
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid in sorted(matches):
                _kill_pid_or_group(pid, sig)
            _wait_gone(matches, 8.0 if sig == signal.SIGTERM else 2.0)
        return len(matches)

    def _stop_active_plan(self) -> dict:
        active_pointer = self._active_pointer() or {}
        manifest, manifest_path = self._active_manifest()
        results_root = Path(str(active_pointer.get("results_root") or (manifest.get("results_root") if manifest else ""))).resolve() if (active_pointer or manifest) else None
        runtime = runtime_load_json(runtime_state_path(results_root)) if results_root else None
        pid = runtime.get("pid") if runtime else None

        stopped = False
        if is_pid_alive(pid, cmdline_contains="run_simulation_plan.py"):
            try:
                os.kill(int(pid), signal.SIGTERM)
                stopped = True
            except Exception:
                pass
            deadline = time.time() + 12.0
            while time.time() < deadline and is_pid_alive(pid, cmdline_contains="run_simulation_plan.py"):
                time.sleep(0.25)
            if is_pid_alive(pid, cmdline_contains="run_simulation_plan.py"):
                try:
                    os.kill(int(pid), signal.SIGKILL)
                    stopped = True
                except Exception:
                    pass

        killed_related = self._kill_related_plan_processes(results_root, manifest)

        if manifest:
            max_instance = 8
            process_hints: list[str] = []
            build_dir = Path(str(manifest.get("build_dir") or "")).resolve() if str(manifest.get("build_dir") or "").strip() else None
            for task in manifest.get("tasks", []):
                workers = int(task.get("workers_effective") or 8)
                instance_base = int(task.get("instance_base") or 0)
                max_instance = max(max_instance, instance_base + workers + 4)
                if build_dir is not None:
                    process_hints.append(str(build_dir / "pool_instances_plan" / str(task.get("task_id") or "")))
            cleanup_runtime_slots(
                self.root_dir.parents[1],
                instance_ids=range(0, max(64, max_instance)),
                ports=range(4560, 4761),
                process_hints=process_hints,
            )

        if results_root:
            runtime_payload = runtime_load_json(runtime_state_path(results_root)) or {}
            runtime_payload.update({
                "status": "stopped",
                "updated_at": time.time(),
                "finished_at": time.time(),
            })
            runtime_state_path(results_root).write_text(json.dumps(runtime_payload, indent=2), encoding="utf-8")

        try:
            (self.root_dir / "active_plan.json").unlink(missing_ok=True)
        except Exception:
            pass

        return {
            "ok": True,
            "message": "active plan stopped" if (stopped or killed_related) else "no active plan runtime",
        }

    def _start_plan(self, body: dict) -> dict:
        active_manifest, _ = self._active_manifest()
        if active_manifest:
            runtime = runtime_load_json(runtime_state_path(Path(active_manifest["results_root"]).resolve())) or {}
            if is_pid_alive(runtime.get("pid"), cmdline_contains="run_simulation_plan.py"):
                return {"ok": False, "message": "an active plan is already running"}

        plan_path_raw = str(body.get("plan_path") or "").strip()
        if not plan_path_raw:
            return {"ok": False, "message": "plan_path is required"}
        plan_path = Path(plan_path_raw)
        if not plan_path.is_absolute():
            plan_path = (self.root_dir.parents[1] / plan_path).resolve()
        else:
            plan_path = plan_path.resolve()
        if not plan_path.exists():
            return {"ok": False, "message": f"plan file not found: {plan_path}"}

        cmd = [
            sys.executable,
            str(self.root_dir / "run_simulation_plan.py"),
            "--plan", str(plan_path),
            "--detach",
        ]
        if bool(body.get("clean", True)):
            cmd.append("--clean")
        if bool(body.get("serve_dashboard", True)):
            cmd.append("--serve-dashboard")

        proc = subprocess.run(
            cmd,
            cwd=str(self.root_dir.parents[1]),
            capture_output=True,
            text=True,
            timeout=20.0,
        )
        if proc.returncode != 0:
            return {
                "ok": False,
                "message": (proc.stderr or proc.stdout or "plan start failed").strip(),
            }
        payload = {}
        try:
            payload = json.loads(proc.stdout.strip() or "{}")
        except Exception:
            payload = {"raw": proc.stdout.strip()}
        return {
            "ok": True,
            "message": "plan started",
            "launch": payload,
        }

    def do_GET(self):  # noqa: N802 - stdlib signature
        parsed = urlparse(self.path)
        if parsed.path == "/api/dashboard_state":
            q = parse_qs(parsed.query)
            results_root_raw = q.get("results_root", [""])[0].strip() or None
            self._send_json(self._dashboard_state(results_root_raw))
            return
        if parsed.path == "/api/catalog":
            q = parse_qs(parsed.query)
            rootfs_raw = q.get("rootfs", [""])[0].strip()
            if rootfs_raw:
                rootfs = Path(rootfs_raw)
                if not rootfs.is_absolute():
                    rootfs = (self.root_dir.parents[1] / rootfs).resolve()
                else:
                    rootfs = rootfs.resolve()
            else:
                rootfs = (self.root_dir.parents[1] / "build" / "px4_sitl_default" / "rootfs").resolve()
            self._send_json({
                "ok": True,
                **catalog_payload(self.root_dir, rootfs),
            })
            return
        if parsed.path == "/api/trajectory_points":
            q = parse_qs(parsed.query)
            rootfs_raw = q.get("rootfs", [""])[0].strip()
            if rootfs_raw:
                rootfs = Path(rootfs_raw)
                if not rootfs.is_absolute():
                    rootfs = (self.root_dir.parents[1] / rootfs).resolve()
                else:
                    rootfs = rootfs.resolve()
            else:
                rootfs = (self.root_dir.parents[1] / "build" / "px4_sitl_default" / "rootfs").resolve()
            try:
                traj_id = int(q.get("traj_id", ["0"])[0].strip())
                max_points = max(1, int(q.get("max_points", ["600"])[0].strip()))
                self._send_json({
                    "ok": True,
                    "traj_id": traj_id,
                    "rootfs": str(rootfs),
                    "points": trajectory_points(rootfs, traj_id, max_points=max_points),
                })
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
            return
        if parsed.path == "/api/plan_runs":
            self._send_json({
                "ok": True,
                "plans": self._available_plans(),
            })
            return
        if parsed.path == "/api/load_plan":
            q = parse_qs(parsed.query)
            raw_path = q.get("path", [""])[0].strip()
            try:
                payload = load_plan_yaml(self.root_dir, raw_path)
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
                return
            self._send_json({
                "ok": True,
                **payload,
            })
            return
        if parsed.path == "/api/task_history":
            q = parse_qs(parsed.query)
            task_id = q.get("task_id", [""])[0].strip() or None
            results_dir_raw = q.get("results_dir", [""])[0].strip() or None
            results_root_raw = q.get("results_root", [""])[0].strip() or None
            manifest, _ = self._selected_manifest(results_root_raw)
            results_dir, task = self._results_dir_from_request(results_dir_raw, manifest, task_id)
            rows = self._load_history(results_dir)
            self._send_json({
                "ok": True,
                "task_id": task_id or (task.get("task_id") if task else "legacy"),
                "results_dir": str(results_dir),
                "run_config": self._read_json(results_dir / "run_config.json") or {},
                "pool_status": self._read_json(results_dir / "pool_status.json") or {},
                "pool_status_mtime": self._file_mtime(results_dir / "pool_status.json"),
                "history_mtime": self._file_mtime(results_dir / "history.jsonl"),
                "records": rows,
            })
            return
        if parsed.path == "/api/trace_from_eval":
            q = parse_qs(parsed.query)
            raw = q.get("eval_index", [""])[0]
            task_id = q.get("task_id", [""])[0].strip() or None
            results_dir_raw = q.get("results_dir", [""])[0].strip() or None
            results_root_raw = q.get("results_root", [""])[0].strip() or None
            try:
                eval_index = int(raw)
            except ValueError:
                self._send_json({"ok": False, "message": "invalid eval_index"}, status=400)
                return
            manifest, _ = self._selected_manifest(results_root_raw)
            results_dir, _ = self._results_dir_from_request(results_dir_raw, manifest, task_id)
            self._send_json(self._trace_from_eval(results_dir, eval_index))
            return
        if parsed.path == "/api/live_trace":
            q = parse_qs(parsed.query)
            task_id = q.get("task_id", [""])[0].strip() or None
            worker_raw = q.get("worker_id", [""])[0].strip()
            results_dir_raw = q.get("results_dir", [""])[0].strip() or None
            results_root_raw = q.get("results_root", [""])[0].strip() or None
            worker_id = int(worker_raw) if worker_raw else None
            manifest, _ = self._selected_manifest(results_root_raw)
            results_dir, _ = self._results_dir_from_request(results_dir_raw, manifest, task_id)
            self._send_json(self._live_trace(results_dir, worker_id=worker_id))
            return
        if parsed.path == "/api/replay_trace":
            self._send_json(self._replay_trace())
            return
        if parsed.path == "/api/replay_status":
            self._send_json(self._replay_status())
            return

        self._normalize_path()
        return super().do_GET()

    def do_POST(self):  # noqa: N802 - stdlib signature
        parsed = urlparse(self.path)
        if parsed.path == "/api/save_plan":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
                filename = str(payload.get("filename") or "").strip()
                plan_payload = payload.get("plan")
                if not isinstance(plan_payload, dict):
                    raise RuntimeError("plan must be a JSON object")
                out_path = save_plan_yaml(self.root_dir, filename, plan_payload)
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
                return
            self._send_json({
                "ok": True,
                "message": "plan saved",
                "path": str(out_path),
            })
            return
        if parsed.path == "/api/start_plan":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
                result = self._start_plan(payload)
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
                return
            self._send_json(result, status=200 if result.get("ok") else 409)
            return
        if parsed.path == "/api/parse_plan":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
                text = str(payload.get("yaml_text") or "")
                plan = yaml.safe_load(text) or {}
                if not isinstance(plan, dict):
                    raise RuntimeError("YAML must contain a top-level mapping")
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
                return
            self._send_json({
                "ok": True,
                "plan": plan,
            })
            return
        if parsed.path == "/api/stop_plan":
            self._send_json(self._stop_active_plan())
            return
        if parsed.path == "/api/start_replay":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
                result = self._start_replay(payload)
            except Exception as exc:
                self._send_json({"ok": False, "message": str(exc)}, status=400)
                return
            self._send_json(result, status=200 if result.get("ok") else 409)
            return
        if parsed.path == "/api/stop_replay":
            self._send_json(self._stop_replay())
            return
        self._send_json({"ok": False, "message": "unknown endpoint"}, status=404)

    def do_HEAD(self):  # noqa: N802 - stdlib signature
        self._normalize_path()
        return super().do_HEAD()

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent))
    args = ap.parse_args()

    root = Path(args.root).resolve()
    os.chdir(root)
    DashboardHandler.root_dir = root

    server = ThreadingHTTPServer(("0.0.0.0", args.port), DashboardHandler)
    print(f"Dashboard: http://127.0.0.1:{args.port}/dashboard")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
