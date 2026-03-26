#!/usr/bin/env python3
"""Single-console parallel optimizer with headless PX4 simulator workers."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import shutil
import socket
import subprocess
import sys
import time
from concurrent.futures import FIRST_COMPLETED, as_completed, wait
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

import optimize_dfbc as core
from controller_profiles import (
    bounds_for_profile,
    ensure_default_param_file,
    get_controller_profile,
    param_names,
)
from optimizer_registry import get_optimizer_spec, list_optimizer_names
from parallel_pool import EvalTask, WorkerPool
from simulator_backend import DEFAULT_GZ_VEHICLE, DEFAULT_GZ_WORLD, normalize_simulator_name


def parse_workers(raw: str) -> int:
    if raw == "auto":
        cpu = max(1, os.cpu_count() or 1)
        return cpu
    value = int(raw)
    if value <= 0:
        raise RuntimeError("--workers must be > 0 or 'auto'")
    return value


def port_in_use(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.4)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def maybe_launch_dashboard(root_dir: Path, results_dir: Path, port: int, launch: bool) -> subprocess.Popen | None:
    if not launch:
        return None
    if port_in_use(port):
        print(f"dashboard=http://127.0.0.1:{port}/dashboard existing_server=yes")
        return None

    log_path = results_dir / "dashboard.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(
        [
            sys.executable,
            str(root_dir / "serve_dashboard.py"),
            "--port",
            str(port),
            "--root",
            str(root_dir),
        ],
        cwd=str(root_dir),
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


def compute_eval_timeout(args: argparse.Namespace) -> float:
    if float(args.eval_timeout) > 0:
        return float(args.eval_timeout)
    return min(
        core.MAX_EVAL_TIME_FALLBACK_S,
        1.35 * (2.0 * float(args.takeoff_timeout) + float(args.trajectory_timeout)) + 25.0,
    )


def parse_fixed_params(items: list[str]) -> Dict[str, float]:
    parsed: Dict[str, float] = {}
    for raw in items:
        if "=" not in raw:
            raise RuntimeError(f"Invalid --fixed-param value: {raw}")
        name, value = raw.split("=", 1)
        parsed[name.strip()] = float(value.strip())
    return parsed


def parse_bounds_overrides(items: list[str]) -> Dict[str, Tuple[float, float]]:
    parsed: Dict[str, Tuple[float, float]] = {}
    for raw in items:
        if "=" not in raw or ":" not in raw:
            raise RuntimeError(f"Invalid --bounds-override value: {raw}")
        name, span = raw.split("=", 1)
        lo_raw, hi_raw = span.split(":", 1)
        lo = float(lo_raw.strip())
        hi = float(hi_raw.strip())
        if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
            raise RuntimeError(f"Invalid bound range for {name}: {raw}")
        parsed[name.strip()] = (lo, hi)
    return parsed


def cleanup_results(results_dir: Path, live_trace_json: Path, append_history: bool) -> None:
    if append_history:
        return
    for path in results_dir.glob("eval_*.json"):
        path.unlink(missing_ok=True)
    for name in (
        "history.jsonl",
        "best_progress.jsonl",
        "best.json",
        "pool_status.json",
        "run_config.json",
    ):
        (results_dir / name).unlink(missing_ok=True)
    live_trace_json.unlink(missing_ok=True)
    shutil.rmtree(results_dir / "live_traces", ignore_errors=True)
    shutil.rmtree(results_dir / "worker_logs", ignore_errors=True)
    shutil.rmtree(results_dir / "eval_traces", ignore_errors=True)


def write_eval_file(results_dir: Path, task: EvalTask, result: dict, fixed_params: Dict[str, float]) -> None:
    payload = dict(result)
    payload.setdefault("eval_id", task.eval_index)
    payload.setdefault("controller", "")
    payload["iteration"] = task.iteration
    payload["phase"] = task.phase
    payload["candidate"] = task.candidate
    payload["params"] = task.params
    payload["full_params"] = {**fixed_params, **task.params}
    eval_file = results_dir / f"eval_{task.eval_index:05d}.json"
    eval_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def total_expected_evals(global_iters: int, local_iters: int, dim: int, seed_eval: bool = True) -> int:
    return int((1 if seed_eval else 0) + global_iters + local_iters * (1 + 2 * dim))


def resolve_build_dir(repo_root: Path, rootfs: Path, raw_build_dir: str) -> Path:
    if raw_build_dir:
        return ((repo_root / raw_build_dir).resolve()
                if not Path(raw_build_dir).is_absolute()
                else Path(raw_build_dir).resolve())

    # Default to the build tree that owns the selected rootfs.
    return rootfs.parent.resolve()


def main() -> int:
    ap = argparse.ArgumentParser(description="Parallel hybrid controller optimizer")
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--build-dir", default="")
    ap.add_argument("--params-file", default="")
    ap.add_argument("--controller", choices=("dfbc", "pid", "indi", "mpc", "cmpc", "sysid"), default="dfbc")
    ap.add_argument("--results-dir", default="Tools/optimization/results")
    ap.add_argument("--pool-dir", default="build/px4_sitl_default/pool_instances")
    ap.add_argument("--workers", default="auto")
    ap.add_argument("--instance-base", type=int, default=0)
    ap.add_argument("--tcp-port-base", type=int, default=4560)
    ap.add_argument("--dashboard-port", type=int, default=8090)
    ap.add_argument("--no-dashboard", action="store_true")
    ap.add_argument("--simulator", choices=("gz", "jmavsim"), default="gz")
    ap.add_argument("--simulator-vehicle", default=DEFAULT_GZ_VEHICLE)
    ap.add_argument("--simulator-world", default=DEFAULT_GZ_WORLD)
    ap.add_argument("--iterations", type=int, default=50)
    ap.add_argument("--global-iters", type=int, default=-1)
    ap.add_argument("--global-ratio", type=float, default=0.7)
    ap.add_argument("--learning-rate", type=float, default=0.12)
    ap.add_argument("--fd-eps", type=float, default=0.03)
    ap.add_argument("--grad-clip", type=float, default=3.0)
    ap.add_argument("--bo-init-points", type=int, default=12)
    ap.add_argument("--bo-acq-samples", type=int, default=900)
    ap.add_argument("--bo-length-scale", type=float, default=0.28)
    ap.add_argument("--bo-sigma-f", type=float, default=1.0)
    ap.add_argument("--bo-noise", type=float, default=0.03)
    ap.add_argument("--bo-xi", type=float, default=0.02)
    ap.add_argument("--optimizer", choices=list_optimizer_names(), default="bayes")
    ap.add_argument("--bounds-profile", choices=("safe", "wide"), default="safe")
    ap.add_argument("--eval-timeout", type=float, default=-1.0)
    ap.add_argument("--max-failed-eval-streak", type=int, default=8)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--traj-id", type=int, default=0)
    ap.add_argument("--takeoff-alt", type=float, default=2.0)
    ap.add_argument("--trajectory-timeout", type=float, default=180.0)
    ap.add_argument("--takeoff-timeout", type=float, default=30.0)
    ap.add_argument("--w-track", type=float, default=1.0)
    ap.add_argument("--w-energy", type=float, default=0.05)
    ap.add_argument("--fail-penalty", type=float, default=1e6)
    ap.add_argument("--max-failed-base-streak", type=int, default=3)
    ap.add_argument("--sim-speed-factor", type=float, default=1.0)
    ap.add_argument("--max-evals-per-boot", type=int, default=12)
    ap.add_argument("--landing-mode", choices=("land", "rtl"), default="land")
    ap.add_argument("--trace-window", choices=("offboard",), default="offboard")
    ap.add_argument("--engagement-dwell-s", type=float, default=2.0)
    ap.add_argument("--mission-mode", choices=("trajectory", "identification"), default="trajectory")
    ap.add_argument("--ident-profile", default="hover_thrust")
    ap.add_argument("--base-param-file", default="")
    ap.add_argument("--fixed-param", action="append", default=[])
    ap.add_argument("--bounds-override", action="append", default=[])
    ap.add_argument("--append-history", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    profile = get_controller_profile(args.controller)
    optimizer = get_optimizer_spec(args.optimizer)
    args.simulator = normalize_simulator_name(args.simulator)

    args.rootfs = (repo_root / args.rootfs).resolve() if not Path(args.rootfs).is_absolute() else Path(args.rootfs).resolve()
    build_dir = resolve_build_dir(repo_root, args.rootfs, args.build_dir)
    params_default = ensure_default_param_file(args.rootfs, profile)
    if args.params_file:
        args.params_file = (
            (repo_root / args.params_file).resolve()
            if not Path(args.params_file).is_absolute()
            else Path(args.params_file).resolve()
        )
    else:
        args.params_file = params_default.resolve()
    args.results_dir = (
        (repo_root / args.results_dir).resolve()
        if not Path(args.results_dir).is_absolute()
        else Path(args.results_dir).resolve()
    )
    args.pool_dir = (
        (repo_root / args.pool_dir).resolve()
        if not Path(args.pool_dir).is_absolute()
        else Path(args.pool_dir).resolve()
    )

    args.results_dir.mkdir(parents=True, exist_ok=True)
    args.pool_dir.mkdir(parents=True, exist_ok=True)
    args.live_trace_json = args.results_dir / "live_trace.json"
    fixed_params = parse_fixed_params(args.fixed_param)
    bounds_overrides = parse_bounds_overrides(args.bounds_override)

    worker_count = parse_workers(args.workers)
    bounds = bounds_for_profile(profile, args.bounds_profile)
    unknown_bounds = [name for name in bounds_overrides if name not in bounds]
    if unknown_bounds:
        raise RuntimeError(f"Unknown bound override parameters for {profile.display_name}: {unknown_bounds}")
    bounds.update(bounds_overrides)
    dim = len(param_names(profile))

    if args.iterations <= 0:
        raise RuntimeError("--iterations must be > 0")

    if args.iterations == 1:
        global_iters = 1
    elif args.global_iters >= 1:
        global_iters = max(1, min(args.iterations, args.global_iters))
    else:
        global_iters = int(round(args.iterations * args.global_ratio))
        global_iters = max(1, min(args.iterations - 1, global_iters))
    local_iters = max(0, args.iterations - global_iters)
    seed_eval = True
    total_evals = total_expected_evals(global_iters, local_iters, dim, seed_eval=seed_eval)

    backup_file = args.results_dir / f"{profile.param_file_name}.initial.bak"
    if not backup_file.exists():
        shutil.copyfile(args.params_file, backup_file)

    cleanup_results(args.results_dir, args.live_trace_json, args.append_history)

    history_file = args.results_dir / "history.jsonl"
    best_file = args.results_dir / "best.json"
    best_progress_file = args.results_dir / "best_progress.jsonl"
    run_config_file = args.results_dir / "run_config.json"

    run_config = {
        "rootfs": str(args.rootfs),
        "build_dir": str(build_dir),
        "params_file": str(args.params_file),
        "results_dir": str(args.results_dir),
        "controller": profile.name,
        "controller_display_name": profile.display_name,
        "optimizer": optimizer.name,
        "optimizer_display_name": optimizer.display_name,
        "bounds_profile": args.bounds_profile,
        "iterations": args.iterations,
        "global_iters": global_iters,
        "local_iters": local_iters,
        "learning_rate": args.learning_rate,
        "fd_eps": args.fd_eps,
        "grad_clip": args.grad_clip,
        "bo_init_points": args.bo_init_points,
        "bo_acq_samples": args.bo_acq_samples,
        "bo_length_scale": args.bo_length_scale,
        "bo_sigma_f": args.bo_sigma_f,
        "bo_noise": args.bo_noise,
        "bo_xi": args.bo_xi,
        "eval_timeout": args.eval_timeout,
        "max_failed_eval_streak": args.max_failed_eval_streak,
        "seed": args.seed,
        "traj_id": args.traj_id,
        "takeoff_alt": args.takeoff_alt,
        "trajectory_timeout": args.trajectory_timeout,
        "takeoff_timeout": args.takeoff_timeout,
        "w_track": args.w_track,
        "w_energy": args.w_energy,
        "fail_penalty": args.fail_penalty,
        "sim_speed_factor": args.sim_speed_factor,
        "simulator": args.simulator,
        "simulator_vehicle": args.simulator_vehicle,
        "simulator_world": args.simulator_world,
        "max_evals_per_boot": args.max_evals_per_boot,
        "landing_mode": args.landing_mode,
        "trace_window": args.trace_window,
        "engagement_dwell_s": args.engagement_dwell_s,
        "mission_mode": args.mission_mode,
        "ident_profile": args.ident_profile,
        "base_param_file": args.base_param_file,
        "max_failed_base_streak": args.max_failed_base_streak,
        "append_history": args.append_history,
        "workers": worker_count,
        "instance_base": args.instance_base,
        "tcp_port_base": args.tcp_port_base,
        "seed_eval": seed_eval,
        "total_expected_evals": total_evals,
        "param_names": param_names(profile),
        "fixed_params": fixed_params,
        "bounds": {name: [float(pair[0]), float(pair[1])] for name, pair in bounds.items()},
        "formula": "fitness = w_track * track_rmse + w_energy * energy_term",
        "global_method": optimizer.global_method,
        "local_method": optimizer.local_method,
    }
    run_config_file.write_text(json.dumps(run_config, indent=2), encoding="utf-8")

    maybe_launch_dashboard(
        tool_root,
        args.results_dir,
        args.dashboard_port,
        launch=not args.no_dashboard,
    )

    lo, hi = core.bounds_array(profile, bounds)
    rng = np.random.default_rng(args.seed)
    params0 = core.apply_param_rules(profile, core.parse_param_file(args.params_file, profile), bounds)
    vec0 = core.params_to_vec(profile, params0)

    eval_args = {
        "traj_id": args.traj_id,
        "takeoff_alt": args.takeoff_alt,
        "trajectory_timeout": args.trajectory_timeout,
        "takeoff_timeout": args.takeoff_timeout,
        "w_track": args.w_track,
        "w_energy": args.w_energy,
        "fixed_params": fixed_params,
        "landing_mode": args.landing_mode,
        "trace_window": args.trace_window,
        "engagement_dwell_s": args.engagement_dwell_s,
        "mission_mode": args.mission_mode,
        "ident_profile": args.ident_profile,
        "base_param_file": args.base_param_file,
    }
    eval_timeout_s = compute_eval_timeout(args)

    pool = WorkerPool(
        repo_root=repo_root,
        build_dir=build_dir,
        base_rootfs=args.rootfs,
        results_dir=args.results_dir,
        pool_dir=args.pool_dir,
        worker_count=worker_count,
        instance_base=args.instance_base,
        tcp_port_base=args.tcp_port_base,
        total_expected_evals=total_evals,
        profile=profile,
        sim_speed_factor=args.sim_speed_factor,
        max_evals_per_boot=args.max_evals_per_boot,
        simulator=args.simulator,
        simulator_vehicle=args.simulator_vehicle,
        simulator_world=args.simulator_world,
    )

    stopped_by_signal = False
    previous_sigint = signal.getsignal(signal.SIGINT)

    def handle_sigint(_signum, _frame) -> None:
        nonlocal stopped_by_signal
        if stopped_by_signal:
            raise SystemExit(130)
        stopped_by_signal = True
        print("interrupt requested, stopping workers...")
        pool.stop()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, handle_sigint)

    best_cost = math.inf
    best_params = dict(params0)
    eval_index = 0
    completed_eval_count = 0
    failed_base_streak = 0
    failed_eval_streak = 0
    run_started = time.time()

    def record_eval(task: EvalTask, result: dict) -> Tuple[float, bool, Dict[str, float]]:
        nonlocal best_cost, best_params, completed_eval_count, failed_eval_streak

        write_eval_file(args.results_dir, task, result, fixed_params)
        cost = float(result["fitness"])
        failed = bool(result.get("failed", False))
        completed_eval_count += 1
        failed_eval_streak = (failed_eval_streak + 1) if failed else 0

        improved = False
        if cost < best_cost:
            best_cost = cost
            best_params = dict(task.params)
            improved = True

        hist_entry = {
            "iteration": task.iteration,
            "phase": task.phase,
            "candidate": task.candidate,
            "eval_index": task.eval_index,
            "fitness": cost,
            "best_so_far": best_cost,
            "failed": failed,
            "error": str(result.get("stderr", ""))[:300],
            "controller": profile.name,
            "worker_id": int(result.get("worker_id", -1)),
            "duration_s": float(result.get("duration_s", 0.0)),
            "params": task.params,
            "full_params": {**fixed_params, **task.params},
        }
        core.append_jsonl(history_file, hist_entry)
        core.report_eval(
            best_progress_file=best_progress_file,
            iteration=task.iteration,
            phase=task.phase,
            candidate=task.candidate,
            eval_index=task.eval_index,
            fitness=cost,
            best_so_far=best_cost,
            failed=failed,
            improved=improved,
        )
        core.write_best_file(
            best_file=best_file,
            iteration=task.iteration,
            phase=task.phase,
            best_cost=best_cost,
            best_params=best_params,
            last_base_cost=cost,
            eval_count=completed_eval_count,
            elapsed_s=time.time() - run_started,
            base_failed=(task.candidate == "base" and failed),
            base_error=str(result.get("stderr", ""))[:300] if failed else "",
        )
        return cost, failed, dict(task.params)

    used_units = {core.quant_key(core.vec_to_unit(vec0, lo, hi))}
    x_obs_unit = []
    y_obs_gp = []
    current_best_unit = core.vec_to_unit(vec0, lo, hi)

    pool.start()

    try:
        seed_task = EvalTask(
            eval_index=eval_index,
            iteration=0,
            phase="seed",
            candidate="start_params",
            params=dict(params0),
        )
        seed_future = pool.submit(seed_task, eval_args, eval_timeout_s, args.fail_penalty)
        eval_index += 1
        seed_result = seed_future.result()
        seed_cost, _seed_failed, used_seed_params = record_eval(seed_task, seed_result)
        x_obs_unit.append(core.vec_to_unit(core.params_to_vec(profile, used_seed_params), lo, hi))
        y_obs_gp.append(math.log1p(min(seed_cost, args.fail_penalty)))
        current_best_unit = core.vec_to_unit(core.params_to_vec(profile, best_params), lo, hi)

        def submit_global_task(cur_it: int):
            nonlocal eval_index

            if optimizer.name == "random":
                cand_unit = core.sample_unique_random(rng, dim, used_units)
                cand_name = "random_global"
            elif optimizer.name == "anneal":
                cand_unit = core.propose_annealed_candidate(
                    rng=rng,
                    incumbent_x=current_best_unit,
                    used=used_units,
                    step_index=cur_it,
                    total_steps=max(1, global_iters),
                )
                cand_name = "anneal_global"
            elif cur_it < max(1, args.bo_init_points) or not x_obs_unit:
                cand_unit = core.sample_unique_random(rng, dim, used_units)
                cand_name = "bo_random"
            else:
                train_x = np.vstack(x_obs_unit)
                train_y = np.array(y_obs_gp, dtype=float)
                cand_unit = core.propose_bo_candidate(
                    rng=rng,
                    train_x=train_x,
                    train_y=train_y,
                    incumbent_x=current_best_unit,
                    used=used_units,
                    acq_samples=args.bo_acq_samples,
                    length_scale=args.bo_length_scale,
                    sigma_f=args.bo_sigma_f,
                    noise=args.bo_noise,
                    xi=args.bo_xi,
                )
                cand_name = "bo_ei"

            cand_vec = core.unit_to_vec(cand_unit, lo, hi)
            cand_params = core.apply_param_rules(profile, core.vec_to_params(profile, cand_vec), bounds)
            task = EvalTask(
                eval_index=eval_index,
                iteration=cur_it,
                phase="global",
                candidate=cand_name,
                params=cand_params,
            )
            future = pool.submit(task, eval_args, eval_timeout_s, args.fail_penalty)
            eval_index += 1
            return task, future

        next_global_it = 0
        inflight_global = {}

        while next_global_it < global_iters and len(inflight_global) < worker_count:
            task, future = submit_global_task(next_global_it)
            inflight_global[future] = task
            next_global_it += 1

        while inflight_global:
            done, _ = wait(list(inflight_global.keys()), return_when=FIRST_COMPLETED)

            for future in done:
                task = inflight_global.pop(future)
                result = future.result()
                cost, _failed, used_params = record_eval(task, result)
                x_obs_unit.append(core.vec_to_unit(core.params_to_vec(profile, used_params), lo, hi))
                y_obs_gp.append(math.log1p(min(cost, args.fail_penalty)))
                current_best_unit = core.vec_to_unit(core.params_to_vec(profile, best_params), lo, hi)

                if next_global_it < global_iters:
                    new_task, new_future = submit_global_task(next_global_it)
                    inflight_global[new_future] = new_task
                    next_global_it += 1

                if failed_eval_streak >= args.max_failed_eval_streak:
                    failed_eval_streak = 0
                    print(
                        f"iter={task.iteration:03d} phase=global warning=failed_eval_streak_recovery "
                        f"best={best_cost:.6f} eval_count={completed_eval_count}"
                    )

                print(
                    f"iter={task.iteration:03d} phase=global inflight={len(inflight_global)} "
                    f"remaining={max(0, global_iters - next_global_it)} "
                    f"best={best_cost:.6f} eval_count={completed_eval_count}"
                )

        local_params = dict(best_params)
        for j in range(local_iters):
            iteration = global_iters + j
            base_task = EvalTask(
                eval_index=eval_index,
                iteration=iteration,
                phase="local",
                candidate="base",
                params=core.apply_param_rules(profile, local_params, bounds),
            )
            base_future = pool.submit(base_task, eval_args, eval_timeout_s, args.fail_penalty)
            eval_index += 1
            base_result = base_future.result()
            base_cost, base_failed, _ = record_eval(base_task, base_result)

            if base_failed:
                failed_base_streak += 1
                local_params = dict(best_params)
                print(
                    f"iter={iteration:03d} phase=local base=FAILED({base_cost:.1f}) "
                    f"best={best_cost:.6f} eval_count={completed_eval_count} recovery=restart_from_best"
                )
                if failed_base_streak >= args.max_failed_base_streak:
                    failed_base_streak = 0
                continue

            failed_base_streak = 0
            perturb_jobs = []
            deltas: Dict[str, Tuple[float, float]] = {}

            for pname in param_names(profile):
                center = local_params[pname]
                delta = core.finite_diff_delta(core.param_spec_map(profile)[pname], center, args.fd_eps)

                p_plus = dict(local_params)
                p_minus = dict(local_params)
                p_plus[pname] = core.clamp(bounds, pname, center + delta)
                p_minus[pname] = core.clamp(bounds, pname, center - delta)
                deltas[pname] = (p_plus[pname] - center, center - p_minus[pname])

                plus_task = EvalTask(
                    eval_index=eval_index,
                    iteration=iteration,
                    phase="local",
                    candidate=f"{pname}+",
                    params=core.apply_param_rules(profile, p_plus, bounds),
                )
                plus_future = pool.submit(plus_task, eval_args, eval_timeout_s, args.fail_penalty)
                eval_index += 1
                perturb_jobs.append((pname, "plus", plus_task, plus_future))

                minus_task = EvalTask(
                    eval_index=eval_index,
                    iteration=iteration,
                    phase="local",
                    candidate=f"{pname}-",
                    params=core.apply_param_rules(profile, p_minus, bounds),
                )
                minus_future = pool.submit(minus_task, eval_args, eval_timeout_s, args.fail_penalty)
                eval_index += 1
                perturb_jobs.append((pname, "minus", minus_task, minus_future))

            per_param: Dict[str, Dict[str, float | bool]] = {}
            future_meta = {future: (pname, sign, task) for pname, sign, task, future in perturb_jobs}
            for future in as_completed(future_meta):
                pname, sign, task = future_meta[future]
                result = future.result()
                cost, failed, _ = record_eval(task, result)
                info = per_param.setdefault(pname, {})
                info[f"{sign}_cost"] = cost
                info[f"{sign}_failed"] = failed

            grad: Dict[str, float] = {}
            for pname in param_names(profile):
                info = per_param[pname]
                delta_plus, delta_minus = deltas[pname]
                grad[pname] = core.central_gradient(
                    base_cost=base_cost,
                    plus_cost=float(info.get("plus_cost", args.fail_penalty)),
                    minus_cost=float(info.get("minus_cost", args.fail_penalty)),
                    delta_plus=delta_plus,
                    delta_minus=delta_minus,
                    plus_failed=bool(info.get("plus_failed", True)),
                    minus_failed=bool(info.get("minus_failed", True)),
                )

            next_params = dict(local_params)
            for pname in param_names(profile):
                g = grad[pname]
                if args.grad_clip > 0:
                    g = max(-args.grad_clip, min(args.grad_clip, g))
                if core.param_spec_map(profile)[pname].kind == "int":
                    g = round(g)
                next_params[pname] = core.clamp(bounds, pname, local_params[pname] - args.learning_rate * g)

            local_params = core.apply_param_rules(profile, next_params, bounds)

            if failed_eval_streak >= args.max_failed_eval_streak:
                local_params = dict(best_params)
                failed_eval_streak = 0
                print(
                    f"iter={iteration:03d} phase=local warning=failed_eval_streak_recovery "
                    f"best={best_cost:.6f} eval_count={completed_eval_count}"
                )

            print(
                f"iter={iteration:03d} phase=local base={base_cost:.6f} "
                f"best={best_cost:.6f} eval_count={completed_eval_count}"
            )

    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        if not stopped_by_signal:
            pool.stop()

    best_params = core.apply_param_rules(profile, best_params, bounds)
    core.write_param_file(
        args.params_file,
        profile,
        best_params,
        header=f"{profile.display_name} parameter set loaded by optimize_dfbc.py",
    )
    print(
        json.dumps(
            {
                "controller": profile.name,
                "best_cost": best_cost,
                "best_params": best_params,
                "fixed_params": fixed_params,
                "full_best_params": {**fixed_params, **best_params},
                "iterations": args.iterations,
                "global_iters": global_iters,
                "local_iters": local_iters,
                "eval_count": completed_eval_count,
                "workers": worker_count,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("interrupted")
        raise SystemExit(130)
