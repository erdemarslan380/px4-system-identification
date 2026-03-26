#!/usr/bin/env python3
"""Helpers for plan runtime state, stale-run detection, and manifest normalization."""

from __future__ import annotations

import copy
import json
import time
from pathlib import Path

STALE_TASK_AGE_S = 120.0


def load_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)


def pid_cmdline(pid: int | None) -> str:
    if pid is None:
        return ""
    try:
        pid_int = int(pid)
    except Exception:
        return ""
    if pid_int <= 0:
        return ""
    path = Path(f"/proc/{pid_int}/cmdline")
    try:
        raw = path.read_bytes()
    except Exception:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", errors="ignore").strip()


def is_pid_alive(pid: int | None, cmdline_contains: str | None = None) -> bool:
    if pid is None:
        return False
    try:
        pid_int = int(pid)
    except Exception:
        return False
    if pid_int <= 0:
        return False
    if not Path(f"/proc/{pid_int}").exists():
        return False
    if not cmdline_contains:
        return True
    cmdline = pid_cmdline(pid_int)
    return bool(cmdline and cmdline_contains in cmdline)


def runtime_state_path(results_root: Path) -> Path:
    return results_root / "plan_runtime.json"


def task_results_dir(manifest: dict, task: dict) -> Path:
    if task.get("results_dir"):
        return Path(str(task["results_dir"])).resolve()
    return (Path(str(manifest["results_root"])) / str(task["task_id"])).resolve()


def task_latest_activity_s(manifest: dict, task: dict, now: float | None = None) -> float | None:
    now = time.time() if now is None else float(now)
    results_dir = task_results_dir(manifest, task)
    candidates = [
        results_dir / "pool_status.json",
        results_dir / "history.jsonl",
        results_dir / "best_progress.jsonl",
        results_dir / "best.json",
    ]
    mtimes = []
    for path in candidates:
        try:
            mtimes.append(path.stat().st_mtime)
        except Exception:
            continue
    if not mtimes:
        return None
    return max(0.0, now - max(mtimes))


def is_task_stale(manifest: dict, task: dict, now: float | None = None, stale_age_s: float = STALE_TASK_AGE_S) -> bool:
    if str(task.get("status", "")) != "running":
        return False
    now = time.time() if now is None else float(now)
    results_root = Path(str(manifest["results_root"])).resolve()
    runtime = load_json(runtime_state_path(results_root)) or {}
    runtime_pid = runtime.get("pid")
    runtime_status = str(runtime.get("status", ""))
    runtime_task_id = str(runtime.get("current_task_id", ""))
    task_id = str(task.get("task_id", ""))

    if runtime_status == "running" and runtime_task_id == task_id and is_pid_alive(runtime_pid, cmdline_contains="run_simulation_plan.py"):
        return False

    age_s = task_latest_activity_s(manifest, task, now=now)
    if age_s is None:
        return True
    return age_s > float(stale_age_s)


def normalize_manifest_runtime(manifest: dict, stale_age_s: float = STALE_TASK_AGE_S) -> tuple[dict, bool]:
    out = copy.deepcopy(manifest)
    changed = False
    now = time.time()
    for index, task in enumerate(out.get("tasks", [])):
        if not is_task_stale(out, task, now=now, stale_age_s=stale_age_s):
            continue
        task["status"] = "failed"
        task["status_detail"] = "interrupted: stale runner detected"
        if not task.get("finished_at"):
            task["finished_at"] = now
        if str(out.get("current_task_id", "")) == str(task.get("task_id", "")):
            out["current_task_id"] = None
            out["current_task_index"] = -1
        changed = True
        break

    if changed:
        tasks = out.get("tasks", [])
        out["counts"] = {
            "pending": sum(1 for task in tasks if task.get("status") == "pending"),
            "running": sum(1 for task in tasks if task.get("status") == "running"),
            "ok": sum(1 for task in tasks if task.get("status") == "ok"),
            "failed": sum(1 for task in tasks if task.get("status") == "failed"),
        }
    return out, changed
