#!/usr/bin/env python3
"""Shared discovery helpers for planner, live dashboard, and review UI."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

from controller_profiles import all_profiles, describe_param
from optimizer_registry import OPTIMIZERS
from simulator_backend import (
    DEFAULT_GZ_VEHICLE,
    DEFAULT_GZ_WORLD,
    DEFAULT_SIMULATOR,
    available_simulators,
    discover_gz_models,
    discover_gz_worlds,
)
from trajectory_utils import (
    recommended_trajectory_timeout_s,
    trajectory_duration_s,
    trajectory_sample_count,
)


UI_DEFAULTS: dict[str, Any] = {
    "workers": "auto",
    "iterations": 10,
    "global_iters": 8,
    "optimizer": "bayes",
    "takeoff_alt": 2.0,
    "takeoff_timeout": 40.0,
    "trajectory_timeout": "auto",
    "w_track": 1.0,
    "w_energy": 0.05,
    "fail_penalty": 1_000_000.0,
    "sim_speed_factor": 1.0,
    "simulator": DEFAULT_SIMULATOR,
    "simulator_vehicle": DEFAULT_GZ_VEHICLE,
    "simulator_world": DEFAULT_GZ_WORLD,
    "max_evals_per_boot": "auto",
    "bounds_profile": "safe",
    "seed": 17,
    "dashboard_port": 8090,
    "landing_mode": "land",
    "trace_window": "offboard",
    "engagement_dwell_s": 2.0,
    "mission_mode": "trajectory",
    "ident_profile": "hover_thrust",
    "base_param_file": "",
}

UI_LIMITS: dict[str, Any] = {
    "seed_min": 0,
    "seed_max": 2_147_483_647,
    "sim_speed_factor_min": 0.1,
    "sim_speed_factor_max": 10.0,
    "workers_min": 1,
    "workers_max": max(1, int(os.cpu_count() or 8)),
    "cpu_count": max(1, int(os.cpu_count() or 8)),
}

IDENTIFICATION_PROFILES = (
    "hover_thrust",
    "roll_sweep",
    "pitch_sweep",
    "yaw_sweep",
    "drag_x",
    "drag_y",
    "drag_z",
    "mass_vertical",
    "motor_step",
)


def _repo_root(tool_root: Path) -> Path:
    return Path(tool_root).resolve().parents[1]


def resolve_plan_path(tool_root: Path, raw_path: str, *, for_write: bool = False) -> Path:
    tool_root = Path(tool_root).resolve()
    repo_root = _repo_root(tool_root)
    text = str(raw_path or "").strip()
    if not text:
        raise RuntimeError("Plan path is required")

    candidate = Path(text)
    if candidate.is_absolute():
        resolved = candidate.resolve()
    elif any(sep in text for sep in ("/", "\\")):
        resolved = (repo_root / candidate).resolve()
    else:
        safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_", ".") else "_" for ch in text).strip("._")
        if not safe_name:
            raise RuntimeError("Invalid plan filename")
        if not safe_name.endswith(".yaml"):
            safe_name += ".yaml"
        resolved = (tool_root / "generated_plans" / safe_name).resolve()

    if resolved != repo_root and repo_root not in resolved.parents:
        raise RuntimeError("Plan path must stay inside the repository")
    if resolved.suffix.lower() not in {".yaml", ".yml"}:
        resolved = resolved.with_suffix(".yaml")
    if for_write:
        resolved.parent.mkdir(parents=True, exist_ok=True)
    return resolved


def _task_history_row_count(results_dir: Path) -> int:
    history_path = results_dir / "history.jsonl"
    if not history_path.exists():
        return 0
    count = 0
    for line in history_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            count += 1
    return count


def discover_trajectories(rootfs: Path) -> list[dict]:
    traj_dir = Path(rootfs).resolve() / "trajectories"
    out: list[dict] = []
    if not traj_dir.exists():
        return out
    for path in sorted(traj_dir.glob("id_*.traj")):
        try:
            traj_id = int(path.stem.split("_", 1)[1])
        except Exception:
            continue
        duration_s = trajectory_duration_s(rootfs, traj_id)
        sample_count = trajectory_sample_count(rootfs, traj_id)
        out.append({
            "traj_id": traj_id,
            "label": f"Trajectory {traj_id}",
            "file": str(path),
            "duration_s": float(duration_s),
            "sample_count": int(sample_count),
            "recommended_timeout_s": float(recommended_trajectory_timeout_s(duration_s)),
        })
    return out


def discover_controllers() -> list[dict]:
    out: list[dict] = []
    for profile in all_profiles():
        out.append({
            "name": profile.name,
            "display_name": profile.display_name,
            "cli_name": profile.cli_name,
            "trace_name": profile.trace_name,
            "controller_id": profile.controller_id,
            "param_file_name": profile.param_file_name,
            "param_dim": len(profile.params),
            "params": [
                {
                    "name": spec.name,
                    "default": float(spec.default),
                    "safe_bounds": [float(spec.safe_bounds[0]), float(spec.safe_bounds[1])],
                    "wide_bounds": [float(spec.wide_bounds[0]), float(spec.wide_bounds[1])],
                    "kind": spec.kind,
                    "description": describe_param(profile.name, spec.name),
                }
                for spec in profile.params
            ],
        })
    out.sort(key=lambda item: item["name"])
    return out


def discover_optimizers() -> list[dict]:
    out: list[dict] = []
    for spec in OPTIMIZERS.values():
        out.append({
            "name": spec.name,
            "display_name": spec.display_name,
            "short_label": spec.short_label,
            "color": spec.color,
            "description": spec.description,
            "global_method": spec.global_method,
            "local_method": spec.local_method,
        })
    out.sort(key=lambda item: item["name"])
    return out


def discover_plan_files(tool_root: Path) -> list[dict]:
    tool_root = Path(tool_root).resolve()
    generated = tool_root / "generated_plans"
    generated.mkdir(parents=True, exist_ok=True)
    seen: set[Path] = set()
    out: list[dict] = []
    candidates = list(tool_root.glob("*.yaml")) + list(generated.glob("*.yaml"))
    for path in sorted(candidates):
        path = path.resolve()
        if path in seen:
            continue
        seen.add(path)
        rel = path.relative_to(tool_root.parents[1]) if path.is_relative_to(tool_root.parents[1]) else path
        out.append({
            "name": path.stem,
            "path": str(path),
            "relative_path": str(rel),
            "generated": generated in path.parents,
            "mtime": path.stat().st_mtime,
        })
    return out


def discover_plan_runs(tool_root: Path) -> list[dict]:
    tool_root = Path(tool_root).resolve()
    out: list[dict] = []
    for manifest_path in sorted((tool_root / "plan_runs").glob("*/plan_manifest.json")):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        counts = manifest.get("counts") or manifest.get("summary") or {}
        out.append({
            "name": str(manifest.get("name") or manifest_path.parent.name),
            "results_root": str(Path(manifest.get("results_root") or manifest_path.parent).resolve()),
            "manifest_path": str(manifest_path.resolve()),
            "plan_file": str(manifest.get("plan_file") or ""),
            "counts": counts,
            "started_at": manifest.get("started_at"),
            "finished_at": manifest.get("finished_at"),
            "current_task_id": manifest.get("current_task_id"),
        })
    out.sort(key=lambda item: float(item.get("finished_at") or item.get("started_at") or 0.0), reverse=True)
    return out


def historical_runtime_stats(tool_root: Path) -> dict:
    tool_root = Path(tool_root).resolve()
    by_controller: dict[str, list[float]] = {}
    by_controller_traj: dict[str, list[float]] = {}
    by_controller_traj_opt: dict[str, list[float]] = {}
    global_samples: list[float] = []
    for run in discover_plan_runs(tool_root):
        manifest_path = Path(run["manifest_path"])
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        for task in manifest.get("tasks", []):
            duration_s = task.get("duration_s")
            status = str(task.get("status") or "")
            if not isinstance(duration_s, (int, float)) or float(duration_s) <= 0:
                continue
            if status not in {"ok", "failed"}:
                continue
            task_id = str(task.get("task_id") or "")
            results_dir = Path(str(task.get("results_dir") or (Path(manifest["results_root"]) / task_id))).resolve()
            eval_count = _task_history_row_count(results_dir)
            if eval_count <= 0:
                continue
            wall = float(duration_s) / float(eval_count)
            controller = str(task.get("controller") or "")
            traj_id = int(task.get("traj_id") or -1)
            optimizer = str(task.get("optimizer") or "bayes")
            by_controller.setdefault(controller, []).append(wall)
            by_controller_traj.setdefault(f"{controller}:{traj_id}", []).append(wall)
            by_controller_traj_opt.setdefault(f"{controller}:{traj_id}:{optimizer}", []).append(wall)
            global_samples.append(wall)

    def summarize(samples: dict[str, list[float]]) -> dict[str, float]:
        out: dict[str, float] = {}
        for key, values in samples.items():
            if values:
                out[key] = float(sum(values) / len(values))
        return out

    return {
        "by_controller": summarize(by_controller),
        "by_controller_traj": summarize(by_controller_traj),
        "by_controller_traj_optimizer": summarize(by_controller_traj_opt),
        "global_wall_per_eval_s": float(sum(global_samples) / len(global_samples)) if global_samples else 65.0,
    }


def estimate_task_duration_s(
    controller: str,
    traj_id: int,
    optimizer: str,
    iterations: int,
    global_iters: int,
    param_dim: int,
    workers: int,
    stats: dict,
) -> dict[str, float]:
    local_iters = max(0, int(iterations) - int(global_iters))
    total_expected_evals = int(1 + int(global_iters) + local_iters * (1 + 2 * int(param_dim)))
    useful_workers = max(1, min(int(workers), max(int(global_iters), min(int(workers), 2 * max(1, int(param_dim))))))
    key_full = f"{controller}:{int(traj_id)}:{optimizer}"
    key_partial = f"{controller}:{int(traj_id)}"
    wall_per_eval = (
        float((stats.get("by_controller_traj_optimizer") or {}).get(key_full)
              or (stats.get("by_controller_traj") or {}).get(key_partial)
              or (stats.get("by_controller") or {}).get(controller)
              or stats.get("global_wall_per_eval_s")
              or 65.0)
    )
    global_component = int(global_iters) * wall_per_eval / max(1, useful_workers)
    local_component = local_iters * wall_per_eval * (1.0 + 2.0 * int(param_dim) / max(1, useful_workers))
    total_s = 1.2 * (global_component + local_component + wall_per_eval)
    return {
        "total_expected_evals": float(total_expected_evals),
        "estimated_wall_s": float(total_s),
        "wall_per_eval_s": float(wall_per_eval),
    }


def catalog_payload(tool_root: Path, rootfs: Path) -> dict:
    tool_root = Path(tool_root).resolve()
    rootfs = Path(rootfs).resolve()
    return {
        "controllers": discover_controllers(),
        "identification_profiles": list(IDENTIFICATION_PROFILES),
        "simulators": available_simulators(),
        "gazebo_models": discover_gz_models(_repo_root(tool_root)),
        "gazebo_worlds": discover_gz_worlds(_repo_root(tool_root)),
        "optimizers": discover_optimizers(),
        "trajectories": discover_trajectories(rootfs),
        "plan_files": discover_plan_files(tool_root),
        "plan_runs": discover_plan_runs(tool_root),
        "history_stats": historical_runtime_stats(tool_root),
        "ui_defaults": dict(UI_DEFAULTS),
        "ui_limits": dict(UI_LIMITS),
        "rootfs": str(rootfs),
    }


def load_plan_yaml(tool_root: Path, filename: str) -> dict:
    path = resolve_plan_path(tool_root, filename, for_write=False)
    if not path.exists():
        raise RuntimeError(f"Plan file not found: {path}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise RuntimeError("Plan file must contain a top-level mapping")
    return {
        "path": str(path),
        "relative_path": str(path.relative_to(_repo_root(Path(tool_root).resolve()))) if path.is_relative_to(_repo_root(Path(tool_root).resolve())) else str(path),
        "plan": payload,
    }


def save_plan_yaml(tool_root: Path, filename: str, payload: dict) -> Path:
    path = resolve_plan_path(tool_root, filename, for_write=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=False), encoding="utf-8")
    return path
