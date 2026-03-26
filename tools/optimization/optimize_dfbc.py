#!/usr/bin/env python3
"""Controller optimizer with selectable global search and central-difference refinement."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from controller_profiles import (
    ControllerProfile,
    ParamSpec,
    apply_param_rules,
    bounds_array,
    bounds_for_profile,
    default_params,
    ensure_default_param_file,
    finite_diff_delta as profile_finite_diff_delta,
    get_controller_profile,
    param_names,
    param_spec_map,
    params_to_vec,
    parse_param_file,
    vec_to_params,
    write_param_file,
)
from optimizer_registry import get_optimizer_spec, list_optimizer_names

MAX_EVAL_TIME_FALLBACK_S = 300.0
DEFAULT_PROFILE = get_controller_profile("dfbc")
DFBC_PARAM_NAMES = param_names(DEFAULT_PROFILE)
WIDE_BOUNDS = bounds_for_profile(DEFAULT_PROFILE, "wide")
SAFE_BOUNDS = bounds_for_profile(DEFAULT_PROFILE, "safe")


def run_eval(
    eval_script: Path,
    args: argparse.Namespace,
    profile: ControllerProfile,
    eval_index: int,
) -> Dict:
    eval_json = args.results_dir / f"eval_{eval_index:05d}.json"
    cmd = [
        sys.executable,
        str(eval_script),
        "--rootfs", str(args.rootfs),
        "--params-file", str(args.params_file),
        "--controller", profile.name,
        "--traj-id", str(args.traj_id),
        "--takeoff-alt", str(args.takeoff_alt),
        "--trajectory-timeout", str(args.trajectory_timeout),
        "--takeoff-timeout", str(args.takeoff_timeout),
        "--w-track", str(args.w_track),
        "--w-energy", str(args.w_energy),
        "--mission-mode", str(args.mission_mode),
        "--ident-profile", str(args.ident_profile),
        "--shell-endpoint", args.shell_endpoint,
        "--telemetry-endpoint", args.telemetry_endpoint,
        "--result-json", str(eval_json),
        "--live-trace-json", str(args.live_trace_json),
        "--eval-id", str(eval_index),
    ]
    if str(args.base_param_file).strip():
        cmd.extend(["--base-param-file", str(args.base_param_file)])

    if args.hard_reset_start:
        cmd.append("--hard-reset-start")
    if args.hard_reset_end:
        cmd.append("--hard-reset-end")

    eval_timeout = (
        float(args.eval_timeout)
        if float(args.eval_timeout) > 0
        else min(
            MAX_EVAL_TIME_FALLBACK_S,
            1.35 * (2.0 * float(args.takeoff_timeout) + float(args.trajectory_timeout)) + 25.0,
        )
    )

    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=eval_timeout)
    except subprocess.TimeoutExpired as exc:
        stderr = (exc.stderr or "").strip() if isinstance(exc.stderr, str) else ""
        return {
            "fitness": float(args.fail_penalty),
            "track_rmse": float(args.fail_penalty),
            "energy_term": 0.0,
            "tracking_log": "",
            "params_file": str(args.params_file),
            "traj_id": args.traj_id,
            "controller": profile.name,
            "failed": True,
            "stderr": f"eval timeout after {eval_timeout:.1f}s; {stderr}".strip(),
        }
    if proc.returncode != 0:
        return {
            "fitness": float(args.fail_penalty),
            "track_rmse": float(args.fail_penalty),
            "energy_term": 0.0,
            "tracking_log": "",
            "params_file": str(args.params_file),
            "traj_id": args.traj_id,
            "controller": profile.name,
            "failed": True,
            "stderr": proc.stderr.strip(),
        }

    if eval_json.exists():
        result = json.loads(eval_json.read_text(encoding="utf-8"))
    else:
        result = json.loads(proc.stdout.strip().splitlines()[-1])

    result["failed"] = False
    result["stderr"] = ""
    result["controller"] = profile.name
    return result


def clamp(bounds: Dict[str, Tuple[float, float]], name: str, value: float) -> float:
    lo, hi = bounds[name]
    return max(lo, min(hi, value))


def append_jsonl(path: Path, obj: Dict) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj) + "\n")


def report_eval(best_progress_file: Path,
                iteration: int,
                phase: str,
                candidate: str,
                eval_index: int,
                fitness: float,
                best_so_far: float,
                failed: bool,
                improved: bool) -> None:
    entry = {
        "iteration": iteration,
        "phase": phase,
        "candidate": candidate,
        "eval_index": eval_index,
        "fitness": fitness,
        "best_so_far": best_so_far,
        "failed": failed,
        "improved": improved,
    }
    append_jsonl(best_progress_file, entry)

    if failed:
        print(
            f"eval={eval_index:05d} iter={iteration:03d} phase={phase:<6} "
            f"cand={candidate:<18} fit=FAILED({fitness:.1f}) best={best_so_far:.6f}"
        )
    else:
        tag = " NEW_BEST" if improved else ""
        print(
            f"eval={eval_index:05d} iter={iteration:03d} phase={phase:<6} "
            f"cand={candidate:<18} fit={fitness:.6f} best={best_so_far:.6f}{tag}"
        )


def vec_to_unit(vec: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    span = np.maximum(1e-12, hi - lo)
    return np.clip((vec - lo) / span, 0.0, 1.0)


def unit_to_vec(unit: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.clip(lo + unit * (hi - lo), lo, hi)


def quant_key(unit: np.ndarray, bins: int = 10000) -> Tuple[int, ...]:
    return tuple(np.clip(np.floor(unit * bins), 0, bins).astype(int).tolist())


def sample_unique_random(rng: np.random.Generator,
                         dim: int,
                         used: set[Tuple[int, ...]]) -> np.ndarray:
    for _ in range(200):
        x = rng.random(dim)
        k = quant_key(x)
        if k not in used:
            used.add(k)
            return x
    x = rng.random(dim)
    used.add(quant_key(x))
    return x


def rbf_kernel(xa: np.ndarray, xb: np.ndarray, length_scale: float, sigma_f: float) -> np.ndarray:
    ls = max(1e-6, float(length_scale))
    dif = (xa[:, None, :] - xb[None, :, :]) / ls
    sq = np.sum(dif * dif, axis=2)
    return (sigma_f ** 2) * np.exp(-0.5 * sq)


def gp_predict(train_x: np.ndarray,
               train_y: np.ndarray,
               test_x: np.ndarray,
               length_scale: float,
               sigma_f: float,
               noise: float) -> Tuple[np.ndarray, np.ndarray]:
    if train_x.ndim != 2 or test_x.ndim != 2:
        raise RuntimeError("Invalid GP input dimensions")

    k_xx = rbf_kernel(train_x, train_x, length_scale, sigma_f)
    diag_noise = max(1e-12, noise * noise)
    eye = np.eye(k_xx.shape[0], dtype=float)
    k_xs = rbf_kernel(train_x, test_x, length_scale, sigma_f)
    k_ss_diag = np.full(test_x.shape[0], sigma_f ** 2, dtype=float)

    last_exc: Exception | None = None
    for jitter in (1e-9, 1e-7, 1e-5, 1e-3):
        try:
            l = np.linalg.cholesky(k_xx + (diag_noise + jitter) * eye)
            alpha = np.linalg.solve(l.T, np.linalg.solve(l, train_y))
            mu = k_xs.T @ alpha
            v = np.linalg.solve(l, k_xs)
            var = np.maximum(1e-12, k_ss_diag - np.sum(v * v, axis=0))
            return mu, var
        except np.linalg.LinAlgError as exc:
            last_exc = exc

    raise RuntimeError(f"GP cholesky failed: {last_exc}")


def normal_cdf_vec(z: np.ndarray) -> np.ndarray:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    return 0.5 * (1.0 + np.array([math.erf(float(v) * inv_sqrt2) for v in z], dtype=float))


def expected_improvement_min(mu: np.ndarray, sigma: np.ndarray, best_y: float, xi: float) -> np.ndarray:
    sigma_safe = np.maximum(1e-12, sigma)
    improvement = best_y - mu - xi
    z = improvement / sigma_safe
    cdf = normal_cdf_vec(z)
    pdf = np.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)
    ei = improvement * cdf + sigma_safe * pdf
    ei[sigma <= 1e-12] = 0.0
    return np.maximum(0.0, ei)


def propose_bo_candidate(rng: np.random.Generator,
                         train_x: np.ndarray,
                         train_y: np.ndarray,
                         incumbent_x: np.ndarray,
                         used: set[Tuple[int, ...]],
                         acq_samples: int,
                         length_scale: float,
                         sigma_f: float,
                         noise: float,
                         xi: float) -> np.ndarray:
    dim = train_x.shape[1]
    n = max(128, int(acq_samples))

    pool = [rng.random((n, dim))]
    for sigma in (0.02, 0.05, 0.1, 0.18):
        local = incumbent_x + sigma * rng.normal(size=(max(32, n // 6), dim))
        pool.append(np.clip(local, 0.0, 1.0))
    x_cand = np.vstack(pool)

    mu, var = gp_predict(
        train_x=train_x,
        train_y=train_y,
        test_x=x_cand,
        length_scale=length_scale,
        sigma_f=sigma_f,
        noise=noise,
    )
    sigma = np.sqrt(np.maximum(1e-12, var))
    best_y = float(np.min(train_y))
    ei = expected_improvement_min(mu, sigma, best_y=best_y, xi=xi)
    order = np.argsort(-ei)

    for idx in order:
        x = x_cand[int(idx)]
        k = quant_key(x)
        if k not in used:
            used.add(k)
            return x

    return sample_unique_random(rng, dim, used)


def propose_annealed_candidate(rng: np.random.Generator,
                               incumbent_x: np.ndarray,
                               used: set[Tuple[int, ...]],
                               step_index: int,
                               total_steps: int) -> np.ndarray:
    dim = incumbent_x.shape[0]
    progress = 0.0 if total_steps <= 1 else max(0.0, min(1.0, step_index / float(total_steps - 1)))
    sigma = max(0.04, 0.28 * (1.0 - progress) + 0.04)

    for _ in range(256):
        if rng.random() < 0.22:
            x = rng.random(dim)
        else:
            x = np.clip(incumbent_x + sigma * rng.normal(size=dim), 0.0, 1.0)
        k = quant_key(x)
        if k not in used:
            used.add(k)
            return x

    return sample_unique_random(rng, dim, used)


def finite_diff_delta(spec: ParamSpec, center: float, fd_eps: float) -> float:
    return profile_finite_diff_delta(spec, center, fd_eps)


def central_gradient(base_cost: float,
                     plus_cost: float,
                     minus_cost: float,
                     delta_plus: float,
                     delta_minus: float,
                     plus_failed: bool,
                     minus_failed: bool) -> float:
    if (not plus_failed) and (not minus_failed):
        denom = max(1e-9, delta_plus + delta_minus)
        return (plus_cost - minus_cost) / denom

    if (not plus_failed) and delta_plus > 1e-9:
        return (plus_cost - base_cost) / delta_plus

    if (not minus_failed) and delta_minus > 1e-9:
        return (base_cost - minus_cost) / delta_minus

    return 0.0


def write_best_file(best_file: Path,
                    iteration: int,
                    phase: str,
                    best_cost: float,
                    best_params: Dict[str, float],
                    last_base_cost: float,
                    eval_count: int,
                    elapsed_s: float,
                    base_failed: bool = False,
                    base_error: str = "") -> None:
    payload = {
        "iteration": iteration,
        "phase": phase,
        "best_so_far": best_cost,
        "best_params": best_params,
        "last_base_cost": last_base_cost,
        "eval_count": eval_count,
        "elapsed_s": elapsed_s,
        "base_failed": base_failed,
    }
    if base_error:
        payload["base_error"] = base_error[:500]
    best_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Controller optimizer with selectable global search and central-difference refinement")
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--params-file", default="")
    ap.add_argument("--controller", choices=("dfbc", "pid", "indi", "mpc", "cmpc", "sysid"), default="dfbc")
    ap.add_argument("--results-dir", default="Tools/optimization/results")
    ap.add_argument("--iterations", type=int, default=50)
    ap.add_argument("--global-iters", type=int, default=-1)
    ap.add_argument("--global-ratio", type=float, default=0.5)
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
    ap.add_argument("--mission-mode", choices=("trajectory", "identification"), default="trajectory")
    ap.add_argument("--ident-profile", default="hover_thrust")
    ap.add_argument("--base-param-file", default="")
    ap.add_argument("--fail-penalty", type=float, default=1e6)
    ap.add_argument("--max-failed-base-streak", type=int, default=3)
    ap.add_argument("--append-history", action="store_true")
    ap.add_argument("--shell-endpoint", default="127.0.0.1:14550")
    ap.add_argument("--telemetry-endpoint", default="127.0.0.1:14550")
    ap.add_argument("--hard-reset-start", action="store_true")
    ap.add_argument("--hard-reset-end", action="store_true")
    args = ap.parse_args()

    profile = get_controller_profile(args.controller)
    optimizer = get_optimizer_spec(args.optimizer)

    args.rootfs = Path(args.rootfs).resolve()
    params_default = ensure_default_param_file(args.rootfs, profile)
    args.params_file = Path(args.params_file).resolve() if args.params_file else params_default.resolve()
    args.results_dir = Path(args.results_dir).resolve()
    args.results_dir.mkdir(parents=True, exist_ok=True)
    args.live_trace_json = args.results_dir / "live_trace.json"
    eval_script = Path(__file__).resolve().parent / "px4_eval.py"

    bounds = bounds_for_profile(profile, args.bounds_profile)

    if args.iterations <= 0:
        raise RuntimeError("--iterations must be > 0")

    if args.iterations == 1:
        global_iters = 1
    elif args.global_iters >= 1:
        global_iters = max(1, min(args.iterations - 1, args.global_iters))
    else:
        global_iters = int(round(args.iterations * args.global_ratio))
        global_iters = max(1, min(args.iterations - 1, global_iters))
    local_iters = max(0, args.iterations - global_iters)

    backup_file = args.results_dir / f"{profile.param_file_name}.initial.bak"
    if not backup_file.exists():
        shutil.copyfile(args.params_file, backup_file)

    history_file = args.results_dir / "history.jsonl"
    best_file = args.results_dir / "best.json"
    best_progress_file = args.results_dir / "best_progress.jsonl"
    run_config_file = args.results_dir / "run_config.json"

    if not args.append_history:
        for p in args.results_dir.glob("eval_*.json"):
            p.unlink(missing_ok=True)
        history_file.unlink(missing_ok=True)
        best_progress_file.unlink(missing_ok=True)
        best_file.unlink(missing_ok=True)
        args.live_trace_json.unlink(missing_ok=True)

    run_config = {
        "rootfs": str(args.rootfs),
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
        "mission_mode": args.mission_mode,
        "ident_profile": args.ident_profile,
        "base_param_file": args.base_param_file,
        "fail_penalty": args.fail_penalty,
        "max_failed_base_streak": args.max_failed_base_streak,
        "append_history": args.append_history,
        "formula": "fitness = w_track * track_rmse + w_energy * energy_term",
        "global_method": optimizer.global_method,
        "local_method": optimizer.local_method,
        "param_names": param_names(profile),
    }
    run_config_file.write_text(json.dumps(run_config, indent=2), encoding="utf-8")

    lo, hi = bounds_array(profile, bounds)
    names = param_names(profile)
    dim = len(names)
    specs = param_spec_map(profile)
    rng = np.random.default_rng(args.seed)

    params0 = apply_param_rules(profile, parse_param_file(args.params_file, profile), bounds)
    vec0 = params_to_vec(profile, params0)
    best_cost = math.inf
    best_params = dict(params0)
    eval_index = 0
    failed_base_streak = 0
    failed_eval_streak = 0

    def evaluate_and_record(iteration: int,
                            phase: str,
                            candidate: str,
                            cand_params: Dict[str, float]) -> Tuple[float, bool, Dict[str, float]]:
        nonlocal best_cost, best_params, eval_index, failed_eval_streak

        cand_params = apply_param_rules(profile, cand_params, bounds)
        write_param_file(
            args.params_file,
            profile,
            cand_params,
            header=f"{profile.display_name} parameter set loaded by optimize_dfbc.py",
        )
        result = run_eval(eval_script, args, profile, eval_index)
        cost = float(result["fitness"])
        failed = bool(result.get("failed", False))
        failed_eval_streak = (failed_eval_streak + 1) if failed else 0

        improved = False
        if cost < best_cost:
            best_cost = cost
            best_params = dict(cand_params)
            improved = True

        hist_entry = {
            "iteration": iteration,
            "phase": phase,
            "candidate": candidate,
            "eval_index": eval_index,
            "fitness": cost,
            "best_so_far": best_cost,
            "failed": failed,
            "error": result.get("stderr", "")[:300],
            "controller": profile.name,
            "params": cand_params,
        }
        append_jsonl(history_file, hist_entry)
        report_eval(
            best_progress_file=best_progress_file,
            iteration=iteration,
            phase=phase,
            candidate=candidate,
            eval_index=eval_index,
            fitness=cost,
            best_so_far=best_cost,
            failed=failed,
            improved=improved,
        )
        eval_index += 1
        return cost, failed, cand_params

    used_units: set[Tuple[int, ...]] = set()
    x_obs_unit: List[np.ndarray] = []
    y_obs_gp: List[float] = []

    current_best_unit = vec_to_unit(vec0, lo, hi)
    used_units.add(quant_key(current_best_unit))

    for it in range(global_iters):
        iter_start = time.time()
        if it == 0:
            cand_unit = current_best_unit.copy()
            cand_name = "start_seed"
        elif optimizer.name == "random":
            cand_unit = sample_unique_random(rng, dim, used_units)
            cand_name = "random_global"
        elif optimizer.name == "anneal":
            cand_unit = propose_annealed_candidate(
                rng=rng,
                incumbent_x=current_best_unit,
                used=used_units,
                step_index=it,
                total_steps=max(1, global_iters),
            )
            cand_name = "anneal_global"
        elif len(x_obs_unit) < max(1, args.bo_init_points):
            cand_unit = sample_unique_random(rng, dim, used_units)
            cand_name = "bo_random"
        else:
            train_x = np.vstack(x_obs_unit)
            train_y = np.array(y_obs_gp, dtype=float)
            cand_unit = propose_bo_candidate(
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

        cand_vec = unit_to_vec(cand_unit, lo, hi)
        cand_params = vec_to_params(profile, cand_vec)
        base_cost, _base_failed, used_params = evaluate_and_record(it, "global", cand_name, cand_params)

        x_obs_unit.append(vec_to_unit(params_to_vec(profile, used_params), lo, hi))
        gp_cost = math.log1p(min(base_cost, args.fail_penalty))
        y_obs_gp.append(gp_cost)

        current_best_unit = vec_to_unit(params_to_vec(profile, best_params), lo, hi)

        if failed_eval_streak >= args.max_failed_eval_streak:
            failed_eval_streak = 0
            print(
                f"iter={it:03d} phase=global warning=failed_eval_streak_recovery "
                f"best={best_cost:.6f} eval_count={eval_index}"
            )

        write_best_file(
            best_file=best_file,
            iteration=it,
            phase="global",
            best_cost=best_cost,
            best_params=best_params,
            last_base_cost=base_cost,
            eval_count=eval_index,
            elapsed_s=time.time() - iter_start,
        )
        print(f"iter={it:03d} base={base_cost:.6f} best={best_cost:.6f} eval_count={eval_index}")

    local_params = dict(best_params)
    for j in range(local_iters):
        iteration = global_iters + j
        iter_start = time.time()
        base_cost, base_failed, _ = evaluate_and_record(iteration, "local", "base", local_params)

        if base_failed:
            failed_base_streak += 1
            local_params = dict(best_params)
            write_best_file(
                best_file=best_file,
                iteration=iteration,
                phase="local",
                best_cost=best_cost,
                best_params=best_params,
                last_base_cost=base_cost,
                eval_count=eval_index,
                elapsed_s=time.time() - iter_start,
                base_failed=True,
                base_error="base evaluation failed",
            )
            print(
                f"iter={iteration:03d} phase=local base=FAILED({base_cost:.1f}) "
                f"best={best_cost:.6f} eval_count={eval_index} recovery=restart_from_best"
            )
            if failed_base_streak >= args.max_failed_base_streak:
                failed_base_streak = 0
            continue

        failed_base_streak = 0
        per_param: Dict[str, Dict[str, float | bool]] = {}
        deltas: Dict[str, Tuple[float, float]] = {}

        for pname in names:
            center = local_params[pname]
            delta = finite_diff_delta(specs[pname], center, args.fd_eps)

            p_plus = dict(local_params)
            p_minus = dict(local_params)
            p_plus[pname] = clamp(bounds, pname, center + delta)
            p_minus[pname] = clamp(bounds, pname, center - delta)
            p_plus = apply_param_rules(profile, p_plus, bounds)
            p_minus = apply_param_rules(profile, p_minus, bounds)
            deltas[pname] = (p_plus[pname] - center, center - p_minus[pname])

            plus_cost, plus_failed, _ = evaluate_and_record(iteration, "local", f"{pname}+", p_plus)
            minus_cost, minus_failed, _ = evaluate_and_record(iteration, "local", f"{pname}-", p_minus)

            per_param[pname] = {
                "plus_cost": plus_cost,
                "minus_cost": minus_cost,
                "plus_failed": plus_failed,
                "minus_failed": minus_failed,
            }

        grad: Dict[str, float] = {}
        for pname in names:
            info = per_param[pname]
            delta_plus, delta_minus = deltas[pname]
            grad[pname] = central_gradient(
                base_cost=base_cost,
                plus_cost=float(info.get("plus_cost", args.fail_penalty)),
                minus_cost=float(info.get("minus_cost", args.fail_penalty)),
                delta_plus=delta_plus,
                delta_minus=delta_minus,
                plus_failed=bool(info.get("plus_failed", True)),
                minus_failed=bool(info.get("minus_failed", True)),
            )

        next_params = dict(local_params)
        for pname in names:
            g = grad[pname]
            if args.grad_clip > 0:
                g = max(-args.grad_clip, min(args.grad_clip, g))
            step = args.learning_rate * g
            if specs[pname].kind == "int":
                step = round(step)
            next_params[pname] = clamp(bounds, pname, local_params[pname] - step)

        local_params = apply_param_rules(profile, next_params, bounds)

        if failed_eval_streak >= args.max_failed_eval_streak:
            local_params = dict(best_params)
            failed_eval_streak = 0
            print(
                f"iter={iteration:03d} phase=local warning=failed_eval_streak_recovery "
                f"best={best_cost:.6f} eval_count={eval_index}"
            )

        write_best_file(
            best_file=best_file,
            iteration=iteration,
            phase="local",
            best_cost=best_cost,
            best_params=best_params,
            last_base_cost=base_cost,
            eval_count=eval_index,
            elapsed_s=time.time() - iter_start,
        )
        print(
            f"iter={iteration:03d} phase=local base={base_cost:.6f} "
            f"best={best_cost:.6f} eval_count={eval_index}"
        )

    best_params = apply_param_rules(profile, best_params, bounds)
    write_param_file(
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
                "iterations": args.iterations,
                "global_iters": global_iters,
                "local_iters": local_iters,
                "eval_count": eval_index,
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
