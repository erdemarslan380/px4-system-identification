#!/usr/bin/env python3
"""Run a YAML-defined sequential tuning plan with one unified dashboard state."""

from __future__ import annotations

import argparse
import json
import os
import signal
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import yaml

from controller_profiles import (
    MPC_SHARED_QR_PARAMS,
    all_profiles,
    default_params,
    ensure_default_param_file,
    get_controller_profile,
    param_names,
    write_param_file,
)
from optimizer_registry import get_optimizer_spec, normalize_optimizer_name
from plan_runtime import normalize_manifest_runtime, runtime_state_path, write_json as write_runtime_json
from runtime_cleanup import cleanup_runtime_slots
from simulator_backend import DEFAULT_GZ_VEHICLE, DEFAULT_GZ_WORLD, DEFAULT_SIMULATOR, normalize_simulator_name
from trajectory_utils import parse_timeout_value, trajectory_duration_s


DEFAULTS: Dict[str, Any] = {
    "workers": "auto",
    "iterations": 10,
    "global_iters": 7,
    "optimizer": "bayes",
    "instance_base": 0,
    "tcp_port_base": 4560,
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

TASK_RUNTIME_KEYS = {
    "status",
    "status_detail",
    "started_at",
    "finished_at",
    "returncode",
    "results_dir",
    "pool_dir",
    "log_path",
    "duration_s",
    "successful_evals",
    "best_cost",
    "best_params",
    "fixed_params_applied",
    "workers_effective",
}


def parse_requested_workers(raw: object) -> int:
    text = str(raw).strip().lower()
    if text == "auto":
        cpu = max(1, os.cpu_count() or 1)
        return cpu
    value = int(text)
    if value <= 0:
        raise RuntimeError(f"Invalid workers value: {raw}")
    return value


def effective_worker_count(task: dict) -> int:
    requested = parse_requested_workers(task.get("workers", "auto"))
    simulator = str(task.get("simulator", "") or "").strip().lower()
    global_iters = int(task.get("global_iters", 0))
    local_iters = int(task.get("local_iters", 0))
    dim = int(task.get("param_dim", 1))

    # Current Gazebo x500 studies are substantially more reliable when each task
    # owns the simulator bootstrap exclusively. Multiple concurrent workers are
    # still supported by the planner, but the runner clamps them here so queued
    # studies do not fail due to sensor/bootstrap races.
    if simulator == "gz":
        return 1

    if local_iters <= 0:
        useful = max(1, global_iters)
    else:
        useful = max(global_iters, min(requested, 2 * max(1, dim)))

    return max(1, min(requested, useful))


def resolve_max_evals_per_boot(controller: str, raw_value: object) -> int:
    text = str(raw_value).strip().lower()
    if not text or text == "auto":
        return 1 if controller in {"pid", "indi"} else 12
    value = int(text)
    if value <= 0:
        raise RuntimeError(f"Invalid max_evals_per_boot value: {raw_value}")
    return value


def parse_status_csv(raw: str) -> set[str]:
    values = {item.strip().lower() for item in str(raw).split(",") if item.strip()}
    return values or {"ok"}


def port_in_use(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.4)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def maybe_launch_dashboard(tool_root: Path, port: int, launch: bool) -> subprocess.Popen | None:
    if not launch:
        return None
    if port_in_use(port):
        print(f"dashboard=http://127.0.0.1:{port}/dashboard existing_server=yes")
        return None

    log_path = tool_root / "results" / "dashboard.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(
        [
            sys.executable,
            str(tool_root / "serve_dashboard.py"),
            "--port",
            str(port),
            "--root",
            str(tool_root),
        ],
        cwd=str(tool_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    deadline = time.time() + 12.0
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"Dashboard server exited early, see {log_path}")
        if port_in_use(port):
            print(f"dashboard=http://127.0.0.1:{port}/dashboard existing_server=no")
            return proc
        time.sleep(0.2)
    raise RuntimeError(f"Dashboard server did not bind port {port}")


def maybe_open_browser(port: int, enabled: bool) -> None:
    if not enabled:
        return
    url = f"http://127.0.0.1:{port}/dashboard"
    cmd = (
        f"sleep 1.5; "
        f"(xdg-open '{url}' >/dev/null 2>&1 || firefox '{url}' >/dev/null 2>&1 || true)"
    )
    subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def resolve_build_dir(repo_root: Path, rootfs: Path, raw_build_dir: str | None) -> Path:
    if raw_build_dir:
        path = Path(str(raw_build_dir))
        return ((repo_root / path).resolve() if not path.is_absolute() else path.resolve())
    return rootfs.parent.resolve()


def prepare_base_rootfs_assets(repo_root: Path, rootfs: Path) -> None:
    rootfs = Path(rootfs).resolve()
    rootfs.mkdir(parents=True, exist_ok=True)
    (rootfs / "parameters").mkdir(parents=True, exist_ok=True)
    (rootfs / "trajectories").mkdir(parents=True, exist_ok=True)
    (rootfs / "tracking_logs").mkdir(parents=True, exist_ok=True)

    source_dir = (repo_root / "trajectories").resolve()
    target_dir = rootfs / "trajectories"

    if not source_dir.exists():
        return

    if list(target_dir.glob("id_*.traj")):
        return

    for traj_file in sorted(source_dir.glob("id_*.traj")):
        shutil.copy2(traj_file, target_dir / traj_file.name)


def load_plan(path: Path) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise RuntimeError("Plan file must contain a mapping at top level")
    if not raw.get("tasks"):
        raise RuntimeError("Plan file must define at least one task")
    return raw


def normalize_task(repo_root: Path, rootfs: Path, raw_task: dict, defaults: dict, index: int) -> dict:
    controller = str(raw_task["controller"]).strip().lower()
    profile = get_controller_profile(controller)
    optimizer = get_optimizer_spec(raw_task.get("optimizer", defaults.get("optimizer", "bayes")))
    simulator = normalize_simulator_name(raw_task.get("simulator", defaults.get("simulator", DEFAULT_SIMULATOR)))
    traj_id = int(raw_task["traj_id"])
    task_id = str(raw_task.get("id", f"{controller}_traj{traj_id}_{index:02d}")).strip()
    task_group_id = str(raw_task.get("task_group_id", raw_task.get("group_id", f"{controller}_traj{traj_id}"))).strip()
    base_label = str(raw_task.get("label", f"{profile.display_name} / Traj {traj_id}")).strip()
    label = f"{base_label} [{optimizer.short_label}]"
    traj_duration = trajectory_duration_s(rootfs, traj_id)
    trajectory_timeout = parse_timeout_value(
        raw_task.get("trajectory_timeout", defaults.get("trajectory_timeout")),
        default_s=float(defaults["trajectory_timeout"]) if str(defaults.get("trajectory_timeout")) != "auto" else 180.0,
        duration_s=traj_duration,
    )
    iterations = int(raw_task.get("iterations", defaults["iterations"]))
    requested_global_iters = int(raw_task.get("global_iters", defaults["global_iters"]))
    if iterations <= 0:
        raise RuntimeError(f"Task {task_id} has invalid iterations={iterations}")
    if iterations == 1:
        global_iters = 1
    else:
        global_iters = max(1, min(iterations, requested_global_iters))
    local_iters = max(0, iterations - global_iters)
    param_dim = len(param_names(profile))
    total_expected_evals = int(1 + global_iters + local_iters * (1 + 2 * param_dim))
    raw_base_param_file = str(raw_task.get("base_param_file", defaults.get("base_param_file", ""))).strip()
    base_param_file = ""
    if raw_base_param_file:
        base_param_file = str(
            ((repo_root / raw_base_param_file).resolve())
            if not Path(raw_base_param_file).is_absolute()
            else Path(raw_base_param_file).resolve()
        )
    task = {
        "task_id": task_id,
        "label": label,
        "controller": controller,
        "task_group_id": task_group_id,
        "optimizer": optimizer.name,
        "optimizer_display_name": optimizer.display_name,
        "optimizer_short_label": optimizer.short_label,
        "optimizer_color": optimizer.color,
        "traj_id": traj_id,
        "iterations": iterations,
        "global_iters": global_iters,
        "local_iters": local_iters,
        "workers": str(raw_task.get("workers", defaults["workers"])),
        "instance_base": int(raw_task.get("instance_base", defaults["instance_base"])),
        "tcp_port_base": int(raw_task.get("tcp_port_base", defaults["tcp_port_base"])),
        "takeoff_alt": float(raw_task.get("takeoff_alt", defaults["takeoff_alt"])),
        "takeoff_timeout": float(raw_task.get("takeoff_timeout", defaults["takeoff_timeout"])),
        "trajectory_timeout": float(trajectory_timeout),
        "trajectory_duration_s": float(traj_duration),
        "w_track": float(raw_task.get("w_track", defaults["w_track"])),
        "w_energy": float(raw_task.get("w_energy", defaults["w_energy"])),
        "fail_penalty": float(raw_task.get("fail_penalty", defaults["fail_penalty"])),
        "sim_speed_factor": float(raw_task.get("sim_speed_factor", defaults["sim_speed_factor"])),
        "simulator": simulator,
        "simulator_vehicle": str(raw_task.get("simulator_vehicle", defaults["simulator_vehicle"])).strip() or DEFAULT_GZ_VEHICLE,
        "simulator_world": str(raw_task.get("simulator_world", defaults["simulator_world"])).strip() or DEFAULT_GZ_WORLD,
        "max_evals_per_boot": resolve_max_evals_per_boot(
            controller,
            raw_task.get("max_evals_per_boot", defaults["max_evals_per_boot"]),
        ),
        "bounds_profile": str(raw_task.get("bounds_profile", defaults["bounds_profile"])),
        "bounds_overrides": {
            str(name): [float(pair[0]), float(pair[1])]
            for name, pair in (raw_task.get("bounds_overrides", {}) or {}).items()
        },
        "seed": int(raw_task.get("seed", defaults["seed"])),
        "landing_mode": str(raw_task.get("landing_mode", defaults["landing_mode"])).strip().lower(),
        "trace_window": str(raw_task.get("trace_window", defaults["trace_window"])).strip().lower(),
        "engagement_dwell_s": float(raw_task.get("engagement_dwell_s", defaults["engagement_dwell_s"])),
        "mission_mode": str(raw_task.get("mission_mode", defaults["mission_mode"])).strip().lower(),
        "ident_profile": str(raw_task.get("ident_profile", defaults["ident_profile"])).strip().lower(),
        "base_param_file": base_param_file,
        "param_names": param_names(profile),
        "param_dim": param_dim,
        "total_expected_evals": total_expected_evals,
        "fixed_params": {
            str(name): float(value)
            for name, value in (raw_task.get("fixed_params", {}) or {}).items()
        },
        "fixed_params_from_task": raw_task.get("fixed_params_from_task"),
        "fixed_params_keys": list(raw_task.get("fixed_params_keys", [])),
        "status": "pending",
        "status_detail": "waiting to start",
        "started_at": None,
        "finished_at": None,
    }
    if task["controller"] == "cmpc" and task["fixed_params_from_task"] and not task["fixed_params_keys"]:
        task["fixed_params_keys"] = list(MPC_SHARED_QR_PARAMS)
    return task


def expand_raw_tasks(raw_tasks: list[dict], defaults: dict) -> list[dict]:
    expanded: list[dict] = []
    for raw_task in raw_tasks:
        controller = str(raw_task["controller"]).strip().lower()
        traj_id = int(raw_task["traj_id"])
        base_id = str(raw_task.get("id", f"{controller}_traj{traj_id}")).strip()
        group_id = str(raw_task.get("group_id", f"{controller}_traj{traj_id}")).strip()
        raw_opts = raw_task.get("optimizers")
        explicit_single_optimizer = raw_opts is None and raw_task.get("optimizer") is not None
        if raw_opts is None:
            default_opts = defaults.get("optimizers")
            if default_opts:
                optimizer_names = [normalize_optimizer_name(str(item)) for item in list(default_opts or [])]
            else:
                raw_opt = raw_task.get("optimizer", defaults.get("optimizer", "bayes"))
                optimizer_names = [normalize_optimizer_name(str(raw_opt))]
        else:
            optimizer_names = [normalize_optimizer_name(str(item)) for item in list(raw_opts or [])]
        for opt_name in optimizer_names:
            clone = dict(raw_task)
            clone["optimizer"] = opt_name
            clone["task_group_id"] = group_id
            if explicit_single_optimizer:
                clone["id"] = base_id
            elif opt_name != "bayes":
                clone["id"] = f"{base_id}__{opt_name}"
            else:
                clone["id"] = base_id
            src = clone.get("fixed_params_from_task")
            if src:
                if explicit_single_optimizer:
                    clone["fixed_params_from_task"] = str(src)
                else:
                    clone["fixed_params_from_task"] = str(src) if opt_name == "bayes" else f"{str(src)}__{opt_name}"
            expanded.append(clone)
    return expanded


def initialize_manifest(repo_root: Path, tool_root: Path, plan_path: Path, raw_plan: dict, args: argparse.Namespace) -> dict:
    rootfs = (repo_root / str(raw_plan.get("rootfs", args.rootfs))).resolve()
    build_dir = resolve_build_dir(repo_root, rootfs, raw_plan.get("build_dir"))
    prepare_base_rootfs_assets(repo_root, rootfs)
    defaults = dict(DEFAULTS)
    defaults.update(raw_plan.get("defaults", {}) or {})
    results_root = raw_plan.get("results_root", f"Tools/optimization/plan_runs/{plan_path.stem}")
    report_html = raw_plan.get("report_html", "Tools/optimization/controller_suite_report.html")
    tasks = [
        normalize_task(repo_root, rootfs, task, defaults, index)
        for index, task in enumerate(expand_raw_tasks(raw_plan["tasks"], defaults))
    ]
    return {
        "name": str(raw_plan.get("name", plan_path.stem)),
        "plan_file": str(plan_path.resolve()),
        "rootfs": str(rootfs),
        "build_dir": str(build_dir),
        "results_root": str((repo_root / results_root).resolve() if not Path(results_root).is_absolute() else Path(results_root).resolve()),
        "report_html": str((repo_root / report_html).resolve() if not Path(report_html).is_absolute() else Path(report_html).resolve()),
        "dashboard_port": int(raw_plan.get("dashboard_port", defaults["dashboard_port"])),
        "started_at": None,
        "finished_at": None,
        "current_task_id": None,
        "current_task_index": -1,
        "tasks": tasks,
    }


def merge_resume_manifest(existing: dict, desired: dict) -> dict:
    desired_tasks = {str(task["task_id"]): task for task in desired.get("tasks", [])}
    existing_tasks = {str(task["task_id"]): task for task in existing.get("tasks", [])}
    merged_tasks: list[dict] = []

    for task_id, desired_task in desired_tasks.items():
        if task_id not in existing_tasks:
            merged_tasks.append(desired_task)
            continue

        old_task = dict(existing_tasks[task_id])
        if str(old_task.get("controller")) != str(desired_task.get("controller")) or int(old_task.get("traj_id")) != int(desired_task.get("traj_id")):
            raise RuntimeError(f"Plan task changed controller/traj for existing task_id={task_id}; choose a new task id")

        merged = dict(old_task)
        for key, value in desired_task.items():
            if old_task.get("status") == "ok" and key not in TASK_RUNTIME_KEYS:
                if key not in merged:
                    merged[key] = value
                continue
            if key in TASK_RUNTIME_KEYS:
                continue
            merged[key] = value
        merged_tasks.append(merged)

    out = dict(existing)
    out["name"] = desired.get("name", existing.get("name"))
    out["plan_file"] = desired.get("plan_file", existing.get("plan_file"))
    out["rootfs"] = desired.get("rootfs", existing.get("rootfs"))
    out["build_dir"] = desired.get("build_dir", existing.get("build_dir"))
    out["results_root"] = desired.get("results_root", existing.get("results_root"))
    out["report_html"] = desired.get("report_html", existing.get("report_html"))
    out["dashboard_port"] = desired.get("dashboard_port", existing.get("dashboard_port"))
    out["tasks"] = merged_tasks

    valid_ids = {str(task["task_id"]) for task in merged_tasks}
    if str(out.get("current_task_id", "")) not in valid_ids:
        out["current_task_id"] = None
        out["current_task_index"] = -1
    return out


def active_pointer_payload(manifest_path: Path, manifest: dict) -> dict:
    return {
        "active": True,
        "manifest_path": str(manifest_path),
        "results_root": manifest["results_root"],
        "report_html": manifest["report_html"],
        "current_task_id": manifest.get("current_task_id"),
        "updated_at": time.time(),
    }


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)


def update_runtime_state(results_root: Path, payload: dict) -> None:
    write_runtime_json(runtime_state_path(results_root), payload)


def update_summary_fields(manifest: dict) -> None:
    tasks = manifest["tasks"]
    counts = {
        "pending": sum(1 for task in tasks if task["status"] == "pending"),
        "running": sum(1 for task in tasks if task["status"] == "running"),
        "ok": sum(1 for task in tasks if task["status"] == "ok"),
        "failed": sum(1 for task in tasks if task["status"] == "failed"),
    }
    manifest["counts"] = counts
    # Keep the legacy summary field populated because the dashboard and older
    # report generators still probe it directly.
    manifest["summary"] = dict(counts)


def resolve_fixed_params(results_root: Path, task: dict) -> dict:
    fixed = dict(task.get("fixed_params", {}))
    source_task_id = task.get("fixed_params_from_task")
    if not source_task_id:
        return fixed

    source_dir = results_root / str(source_task_id)
    best_path = source_dir / "best.json"
    if not best_path.exists():
        raise RuntimeError(f"Fixed params source best.json not found: {best_path}")

    payload = json.loads(best_path.read_text(encoding="utf-8"))
    source_params = payload.get("full_best_params") or payload.get("best_params") or {}
    keys = list(task.get("fixed_params_keys") or [])
    if not keys and task["controller"] == "cmpc":
        keys = list(MPC_SHARED_QR_PARAMS)
    if not keys:
        raise RuntimeError(f"Task {task['task_id']} defines fixed_params_from_task but no fixed_params_keys")
    missing = [name for name in keys if name not in source_params]
    if missing:
        raise RuntimeError(f"Task {task['task_id']} missing fixed params from source: {missing}")
    for name in keys:
        fixed[name] = float(source_params[name])
    return fixed


def count_successful_evals(results_dir: Path) -> int:
    history_path = results_dir / "history.jsonl"
    if not history_path.exists():
        return 0
    count = 0
    for line in history_path.read_text(encoding="utf-8").splitlines():
        raw = line.strip()
        if not raw:
            continue
        row = json.loads(raw)
        if not row.get("failed", False):
            count += 1
    return count


def cleanup_task_runtime(repo_root: Path, build_dir: Path, task: dict) -> None:
    worker_count = effective_worker_count(task)
    instance_base = int(task["instance_base"])
    instance_ids = list(range(instance_base, instance_base + worker_count))
    ports = [int(task["tcp_port_base"]) + instance_id for instance_id in instance_ids]
    pool_hint = build_dir / "pool_instances_plan" / task["task_id"]
    cleanup_runtime_slots(
        repo_root,
        instance_ids=instance_ids,
        ports=ports,
        process_hints=[str(pool_hint)],
    )


def reset_all_controller_defaults(rootfs: Path) -> None:
    for profile in all_profiles():
        params_path = ensure_default_param_file(rootfs, profile)
        write_param_file(
            params_path,
            profile,
            default_params(profile),
            header=f"{profile.display_name} parameter set restored to raw defaults by run_simulation_plan.py",
        )


def run_task(repo_root: Path, tool_root: Path, manifest: dict, task: dict) -> dict:
    orchestrator = tool_root / "orchestrate_parallel.py"
    report_script = tool_root / "generate_controller_suite_report.py"
    results_root = Path(manifest["results_root"])
    results_dir = results_root / task["task_id"]
    build_dir = Path(str(manifest["build_dir"])).resolve()
    pool_dir = build_dir / "pool_instances_plan" / task["task_id"]
    report_html = Path(manifest["report_html"])
    rootfs_path = Path(manifest["rootfs"])
    reset_all_controller_defaults(rootfs_path)
    fixed_params = resolve_fixed_params(results_root, task)
    profile = get_controller_profile(task["controller"])
    params_file = ensure_default_param_file(rootfs_path, profile)
    worker_count = effective_worker_count(task)

    results_dir.mkdir(parents=True, exist_ok=True)
    pool_dir.parent.mkdir(parents=True, exist_ok=True)
    log_path = results_root / f"{task['task_id']}.plan_run.log"
    cleanup_task_runtime(repo_root, build_dir, task)

    cmd = [
        sys.executable,
        "-u",
        str(orchestrator),
        "--controller", task["controller"],
        "--optimizer", task["optimizer"],
        "--rootfs", manifest["rootfs"],
        "--build-dir", str(build_dir),
        "--results-dir", str(results_dir),
        "--pool-dir", str(pool_dir),
        "--workers", str(worker_count),
        "--instance-base", str(task["instance_base"]),
        "--tcp-port-base", str(task["tcp_port_base"]),
        "--iterations", str(task["iterations"]),
        "--global-iters", str(task["global_iters"]),
        "--traj-id", str(task["traj_id"]),
        "--takeoff-alt", str(task["takeoff_alt"]),
        "--takeoff-timeout", str(task["takeoff_timeout"]),
        "--trajectory-timeout", str(task["trajectory_timeout"]),
        "--w-track", str(task["w_track"]),
        "--w-energy", str(task["w_energy"]),
        "--fail-penalty", str(task["fail_penalty"]),
        "--sim-speed-factor", str(task["sim_speed_factor"]),
        "--simulator", str(task["simulator"]),
        "--simulator-vehicle", str(task["simulator_vehicle"]),
        "--simulator-world", str(task["simulator_world"]),
        "--max-evals-per-boot", str(task["max_evals_per_boot"]),
        "--landing-mode", str(task["landing_mode"]),
        "--trace-window", str(task["trace_window"]),
        "--engagement-dwell-s", str(task["engagement_dwell_s"]),
        "--mission-mode", str(task["mission_mode"]),
        "--ident-profile", str(task["ident_profile"]),
        "--bounds-profile", str(task["bounds_profile"]),
        "--seed", str(task["seed"]),
        "--no-dashboard",
    ]
    if str(task.get("base_param_file") or "").strip():
        cmd.extend(["--base-param-file", str(task["base_param_file"])])
    for name, pair in sorted(task.get("bounds_overrides", {}).items()):
        cmd.extend(["--bounds-override", f"{name}={float(pair[0])}:{float(pair[1])}"])
    for name, value in sorted(fixed_params.items()):
        cmd.extend(["--fixed-param", f"{name}={value}"])

    start_time = time.time()
    rc = 1
    try:
        with log_path.open("w", encoding="utf-8") as log_file:
            proc = subprocess.Popen(
                cmd,
                cwd=str(repo_root),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert proc.stdout is not None
            for line in proc.stdout:
                sys.stdout.write(f"[{task['task_id']}] {line}")
                log_file.write(line)
            rc = proc.wait()
    finally:
        cleanup_task_runtime(repo_root, build_dir, task)

    subprocess.run(
        [sys.executable, str(report_script), "--results-root", str(results_root), "--out", str(report_html)],
        cwd=str(repo_root),
        check=False,
    )

    best_path = results_dir / "best.json"
    best_payload = json.loads(best_path.read_text(encoding="utf-8")) if best_path.exists() else {}
    successful = count_successful_evals(results_dir)
    best_cost = best_payload.get("best_cost")
    if best_cost is None:
        best_cost = best_payload.get("best_so_far")
    best_params = best_payload.get("full_best_params") or best_payload.get("best_params") or {}
    return {
        "returncode": rc,
        "results_dir": str(results_dir),
        "pool_dir": str(pool_dir),
        "log_path": str(log_path),
        "duration_s": time.time() - start_time,
        "successful_evals": successful,
        "best_cost": best_cost,
        "best_params": best_params,
        "fixed_params_applied": fixed_params,
        "workers_effective": worker_count,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Run a YAML-defined controller tuning plan")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--serve-dashboard", action="store_true")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--resume-skip-statuses", default="ok")
    ap.add_argument("--resume-include-statuses", default="")
    ap.add_argument("--detach", action="store_true")
    ap.add_argument("--open-browser", action="store_true")
    ap.add_argument("--passive", action="store_true")
    ap.add_argument("--clean", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    plan_path = (repo_root / args.plan).resolve() if not Path(args.plan).is_absolute() else Path(args.plan).resolve()

    raw_plan = load_plan(plan_path)
    manifest = initialize_manifest(repo_root, tool_root, plan_path, raw_plan, args)
    results_root = Path(manifest["results_root"])
    report_html = Path(manifest["report_html"])
    manifest_path = results_root / "plan_manifest.json"
    active_pointer = tool_root / "active_plan.json"

    if args.clean and args.resume:
        raise RuntimeError("--clean and --resume cannot be used together")

    if args.detach:
        results_root.mkdir(parents=True, exist_ok=True)
        log_dir = (tool_root / "results" / "plan_runner_logs").resolve()
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{manifest['name']}.log"
        cmd = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--plan",
            str(plan_path),
            "--rootfs",
            str(args.rootfs),
        ]
        if args.serve_dashboard:
            cmd.append("--serve-dashboard")
        if args.resume:
            cmd.append("--resume")
            if str(args.resume_skip_statuses).strip():
                cmd.extend(["--resume-skip-statuses", str(args.resume_skip_statuses)])
        if str(args.resume_include_statuses).strip():
            cmd.extend(["--resume-include-statuses", str(args.resume_include_statuses)])
        if args.passive:
            cmd.append("--passive")
        if args.clean:
            cmd.append("--clean")
        if args.open_browser:
            maybe_open_browser(int(manifest["dashboard_port"]), True)
        proc = subprocess.Popen(
            cmd,
            cwd=str(repo_root),
            stdout=log_path.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(json.dumps({
            "pid": proc.pid,
            "log": str(log_path),
            "manifest": str(manifest_path),
            "results_root": str(results_root),
        }, indent=2))
        return 0

    if args.clean:
        cleanup_runtime_slots(
            repo_root,
            instance_ids=range(0, max(64, (os.cpu_count() or 1) + 8)),
            ports=range(4560, 4761),
        )
        shutil.rmtree(results_root, ignore_errors=True)
        build_dir_path = Path(str(manifest["build_dir"])).resolve()
        shutil.rmtree(build_dir_path / "pool_instances_plan", ignore_errors=True)
        report_html.unlink(missing_ok=True)
        try:
            active_pointer.unlink(missing_ok=True)
        except Exception:
            pass

    if args.resume and manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest = merge_resume_manifest(manifest, initialize_manifest(repo_root, tool_root, plan_path, raw_plan, args))
    else:
        results_root.mkdir(parents=True, exist_ok=True)
        manifest["started_at"] = time.time()
        update_summary_fields(manifest)
        write_json(manifest_path, manifest)

    manifest, manifest_changed = normalize_manifest_runtime(manifest)
    if manifest_changed:
        write_json(manifest_path, manifest)

    resume_skip_statuses = parse_status_csv(args.resume_skip_statuses)
    resume_include_statuses = parse_status_csv(args.resume_include_statuses) if str(args.resume_include_statuses).strip() else None

    maybe_launch_dashboard(tool_root, manifest["dashboard_port"], args.serve_dashboard)
    maybe_open_browser(int(manifest["dashboard_port"]), args.serve_dashboard and args.open_browser)
    if not args.passive:
        write_json(active_pointer, active_pointer_payload(manifest_path, manifest))
    update_runtime_state(results_root, {
        "pid": os.getpid(),
        "status": "running",
        "plan_file": str(plan_path),
        "results_root": str(results_root),
        "manifest_path": str(manifest_path),
        "current_task_id": manifest.get("current_task_id"),
        "started_at": manifest.get("started_at") or time.time(),
        "updated_at": time.time(),
    })

    report_script = tool_root / "generate_controller_suite_report.py"
    subprocess.run(
        [sys.executable, str(report_script), "--results-root", str(results_root), "--out", str(report_html)],
        cwd=str(repo_root),
        check=False,
    )

    exit_code = 0
    try:
        for index, task in enumerate(manifest["tasks"]):
            if args.resume:
                task_status = str(task.get("status", "")).lower()
                if resume_include_statuses is not None:
                    if task_status not in resume_include_statuses:
                        continue
                elif task_status in resume_skip_statuses:
                    continue
            manifest["current_task_id"] = task["task_id"]
            manifest["current_task_index"] = index
            task["started_at"] = time.time()
            task["status"] = "running"
            task["status_detail"] = "optimizer active"
            task["finished_at"] = None
            task["duration_s"] = None
            task["returncode"] = None
            task["successful_evals"] = 0
            task["best_cost"] = None
            task["best_params"] = {}
            task["fixed_params_applied"] = {}
            task["workers_effective"] = effective_worker_count(task)
            update_summary_fields(manifest)
            write_json(manifest_path, manifest)
            if not args.passive:
                write_json(active_pointer, active_pointer_payload(manifest_path, manifest))
            update_runtime_state(results_root, {
                "pid": os.getpid(),
                "status": "running",
                "plan_file": str(plan_path),
                "results_root": str(results_root),
                "manifest_path": str(manifest_path),
                "current_task_id": task["task_id"],
                "started_at": manifest.get("started_at") or time.time(),
                "updated_at": time.time(),
            })

            result = run_task(repo_root, tool_root, manifest, task)
            task.update(result)
            task["finished_at"] = time.time()
            if result["returncode"] == 0 and int(result["successful_evals"]) > 0:
                task["status"] = "ok"
                best_cost = result.get("best_cost")
                if isinstance(best_cost, (int, float)):
                    task["status_detail"] = f"ok, best={float(best_cost):.6f}, successful_evals={int(result['successful_evals'])}"
                else:
                    task["status_detail"] = f"ok, successful_evals={int(result['successful_evals'])}"
            else:
                task["status"] = "failed"
                task["status_detail"] = f"failed, returncode={result['returncode']}, successful_evals={int(result['successful_evals'])}"
            update_summary_fields(manifest)
            write_json(manifest_path, manifest)
            if not args.passive:
                write_json(active_pointer, active_pointer_payload(manifest_path, manifest))
            update_runtime_state(results_root, {
                "pid": os.getpid(),
                "status": "running",
                "plan_file": str(plan_path),
                "results_root": str(results_root),
                "manifest_path": str(manifest_path),
                "current_task_id": manifest.get("current_task_id"),
                "started_at": manifest.get("started_at") or time.time(),
                "updated_at": time.time(),
            })

        manifest["finished_at"] = time.time()
        manifest["current_task_id"] = None
        manifest["current_task_index"] = -1
        update_summary_fields(manifest)
        write_json(manifest_path, manifest)
        if not args.passive:
            write_json(active_pointer, active_pointer_payload(manifest_path, manifest))

        subprocess.run(
            [sys.executable, str(report_script), "--results-root", str(results_root), "--out", str(report_html)],
            cwd=str(repo_root),
            check=False,
        )
        print(json.dumps({"manifest": str(manifest_path), "report": str(report_html)}, indent=2))
    except Exception:
        exit_code = 1
        raise
    finally:
        update_runtime_state(results_root, {
            "pid": os.getpid(),
            "status": "finished" if exit_code == 0 else "failed",
            "plan_file": str(plan_path),
            "results_root": str(results_root),
            "manifest_path": str(manifest_path),
            "current_task_id": manifest.get("current_task_id"),
            "started_at": manifest.get("started_at") or time.time(),
            "updated_at": time.time(),
            "finished_at": time.time(),
            "exit_code": exit_code,
        })
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
