#!/usr/bin/env python3
"""Benchmark replay determinism and sim-speed sensitivity across controllers."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from controller_profiles import all_profiles, clamp, default_params, get_controller_profile, param_spec_map


def parse_csv_list(text: str, cast=str) -> List:
    values: List = []
    for item in str(text or "").split(","):
        piece = item.strip()
        if not piece:
            continue
        values.append(cast(piece))
    return values


def discover_traj_ids(rootfs: Path) -> List[int]:
    out: List[int] = []
    for path in sorted((rootfs / "trajectories").glob("id_*.traj")):
        try:
            out.append(int(path.stem.split("_", 1)[1]))
        except Exception:
            continue
    return out


def make_param_sets(controller: str, count: int) -> List[Tuple[str, Dict[str, float]]]:
    profile = get_controller_profile(controller)
    specs = list(profile.params)
    params_default = default_params(profile)
    out: List[Tuple[str, Dict[str, float]]] = [("default", dict(params_default))]
    if count <= 1:
        return out

    specs_by_name = param_spec_map(profile)

    def shifted(scale_low: float, scale_high: float) -> Dict[str, float]:
        candidate: Dict[str, float] = {}
        for name, base in params_default.items():
            spec = specs_by_name[name]
            lo, hi = spec.safe_bounds
            if abs(hi - base) >= abs(base - lo):
                raw = base + scale_high * (hi - base)
            else:
                raw = base - scale_low * (base - lo)
            candidate[name] = clamp({name: spec.safe_bounds}, name, raw)
            if spec.kind == "int":
                candidate[name] = float(int(round(candidate[name])))
        return candidate

    out.append(("safe_shift", shifted(0.25, 0.25)))
    if count >= 3:
        spread: Dict[str, float] = {}
        for name, base in params_default.items():
            spec = specs_by_name[name]
            lo, hi = spec.safe_bounds
            if name.endswith("_MIN") or name.endswith("_LOW"):
                raw = base - 0.30 * (base - lo)
            elif name.endswith("_MAX") or name.endswith("_HIGH"):
                raw = base + 0.30 * (hi - base)
            else:
                raw = base + 0.15 * (hi - lo)
            spread[name] = clamp({name: spec.safe_bounds}, name, raw)
            if spec.kind == "int":
                spread[name] = float(int(round(spread[name])))
        out.append(("spread_shift", spread))
    return out[: max(1, count)]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def run_case(
    tool_root: Path,
    repo_root: Path,
    rootfs: Path,
    build_dir: Path,
    run_dir: Path,
    *,
    controller: str,
    traj_id: int,
    params: Dict[str, float],
    sim_speed_factor: float,
    mode: str,
    repeat_index: int,
    takeoff_alt: float,
    takeoff_timeout: float,
    trajectory_timeout: float,
    landing_mode: str,
    engagement_dwell_s: float,
) -> Dict:
    run_dir.mkdir(parents=True, exist_ok=True)
    params_json = json.dumps(params, sort_keys=True)
    result_json = run_dir / "result.json"
    live_trace_json = run_dir / "live_trace.json"
    state_json = run_dir / "state.json"
    log_path = run_dir / "stdout.log"
    instance_id = 40 + (repeat_index % 20)
    tcp_port = 4600 + (repeat_index % 20)
    cmd = [
        sys.executable,
        str(tool_root / "visual_replay.py"),
        "--rootfs", str(rootfs),
        "--build-dir", str(build_dir),
        "--controller", controller,
        "--traj-id", str(traj_id),
        "--params-json", params_json,
        "--fixed-params-json", "{}",
        "--state-json", str(state_json),
        "--result-json", str(result_json),
        "--live-trace-json", str(live_trace_json),
        "--takeoff-alt", str(takeoff_alt),
        "--takeoff-timeout", str(takeoff_timeout),
        "--trajectory-timeout", str(trajectory_timeout),
        "--landing-mode", landing_mode,
        "--trace-window", "offboard",
        "--engagement-dwell-s", str(engagement_dwell_s),
        "--sim-speed-factor", str(sim_speed_factor),
        "--strict-eval",
        "--auto-close-after-finish",
        "--instance-id", str(instance_id),
        "--tcp-port", str(tcp_port),
    ]
    if mode == "headless":
        cmd.append("--headless")

    start = time.time()
    with log_path.open("w", encoding="utf-8") as out:
        proc = subprocess.run(
            cmd,
            cwd=str(repo_root),
            stdout=out,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
            timeout=max(240.0, takeoff_timeout + trajectory_timeout + 120.0),
        )
    wall_s = time.time() - start
    state = json.loads(state_json.read_text(encoding="utf-8")) if state_json.exists() else {}
    result = json.loads(result_json.read_text(encoding="utf-8")) if result_json.exists() else {}
    tracking_src = Path(str(result.get("tracking_log", "")).strip()) if result.get("tracking_log") else None
    tracking_copy = run_dir / "tracking_log.csv"
    tracking_hash = ""
    if tracking_src and tracking_src.exists():
        shutil.copy2(tracking_src, tracking_copy)
        tracking_hash = sha256_file(tracking_copy)
    return {
        "controller": controller,
        "traj_id": traj_id,
        "mode": mode,
        "sim_speed_factor": float(sim_speed_factor),
        "repeat_index": int(repeat_index),
        "returncode": int(proc.returncode),
        "wall_s": float(wall_s),
        "state_status": str(state.get("status") or ""),
        "display_mode": str(state.get("display_mode") or ""),
        "fitness": float(result.get("fitness")) if isinstance(result.get("fitness"), (int, float)) else None,
        "track_rmse": float(result.get("track_rmse")) if isinstance(result.get("track_rmse"), (int, float)) else None,
        "energy_term": float(result.get("energy_term")) if isinstance(result.get("energy_term"), (int, float)) else None,
        "tracking_log_hash": tracking_hash,
        "tracking_log_copy": str(tracking_copy) if tracking_copy.exists() else "",
        "result_json": str(result_json),
        "state_json": str(state_json),
        "stdout_log": str(log_path),
        "error": str(state.get("error") or ""),
    }


def summarize_group(rows: List[Dict]) -> Dict:
    metrics = {}
    for key in ("fitness", "track_rmse", "energy_term", "wall_s"):
        values = [float(r[key]) for r in rows if isinstance(r.get(key), (int, float))]
        if values:
            metrics[key] = {
                "min": min(values),
                "max": max(values),
                "spread": max(values) - min(values),
                "mean": sum(values) / len(values),
            }
    hashes = sorted({str(r.get("tracking_log_hash") or "") for r in rows if r.get("tracking_log_hash")})
    return {
        "runs": len(rows),
        "all_ok": all(int(r.get("returncode", 1)) == 0 and str(r.get("state_status")) == "finished" for r in rows),
        "metrics": metrics,
        "unique_tracking_hashes": hashes,
        "hash_count": len(hashes),
        "display_modes": sorted({str(r.get("display_mode") or "") for r in rows}),
    }


def compare_to_baseline(rows: List[Dict], baseline_rows: List[Dict]) -> Dict:
    if not rows or not baseline_rows:
        return {}
    out: Dict[str, float] = {}
    for key in ("fitness", "track_rmse", "energy_term"):
        vals = [float(r[key]) for r in rows if isinstance(r.get(key), (int, float))]
        base = [float(r[key]) for r in baseline_rows if isinstance(r.get(key), (int, float))]
        if vals and base:
            out[f"{key}_delta_mean"] = (sum(vals) / len(vals)) - (sum(base) / len(base))
    return out


def build_tasks(args: argparse.Namespace, rootfs: Path) -> List[Dict]:
    controllers = parse_csv_list(args.controllers, str) if args.controllers else [p.name for p in all_profiles()]
    traj_ids = parse_csv_list(args.traj_ids, int) if args.traj_ids else discover_traj_ids(rootfs)
    sim_speeds = parse_csv_list(args.sim_speeds, float)
    modes = parse_csv_list(args.modes, str)
    tasks: List[Dict] = []
    for controller in controllers:
        param_sets = make_param_sets(controller, args.param_set_count)
        for traj_id in traj_ids:
            for param_set_name, params in param_sets:
                for mode in modes:
                    for sim_speed in sim_speeds:
                        for repeat_idx in range(args.repeats):
                            tasks.append({
                                "controller": controller,
                                "traj_id": traj_id,
                                "param_set_name": param_set_name,
                                "params": params,
                                "mode": mode,
                                "sim_speed_factor": float(sim_speed),
                                "repeat_index": repeat_idx,
                            })
    return tasks


def main() -> int:
    ap = argparse.ArgumentParser(description="Benchmark replay determinism and sim-speed sensitivity.")
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--build-dir", default="")
    ap.add_argument("--results-root", default="")
    ap.add_argument("--controllers", default="")
    ap.add_argument("--traj-ids", default="")
    ap.add_argument("--sim-speeds", default="0.5,1.0,2.0")
    ap.add_argument("--modes", default="headless,gui")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--param-set-count", type=int, default=2)
    ap.add_argument("--takeoff-alt", type=float, default=2.0)
    ap.add_argument("--takeoff-timeout", type=float, default=40.0)
    ap.add_argument("--trajectory-timeout", type=float, default=180.0)
    ap.add_argument("--landing-mode", choices=("land", "rtl"), default="land")
    ap.add_argument("--engagement-dwell-s", type=float, default=2.0)
    args = ap.parse_args()

    tool_root = Path(__file__).resolve().parent
    repo_root = tool_root.parents[1]
    rootfs = ((repo_root / args.rootfs).resolve() if not Path(args.rootfs).is_absolute() else Path(args.rootfs).resolve())
    build_dir = ((repo_root / args.build_dir).resolve() if args.build_dir and not Path(args.build_dir).is_absolute()
                 else Path(args.build_dir).resolve() if args.build_dir
                 else rootfs.parent.resolve())
    if args.results_root:
        results_root = ((repo_root / args.results_root).resolve() if not Path(args.results_root).is_absolute() else Path(args.results_root).resolve())
    else:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        results_root = (tool_root / "results" / f"determinism_benchmark_{stamp}").resolve()
    results_root.mkdir(parents=True, exist_ok=True)

    tasks = build_tasks(args, rootfs)
    manifest = {
        "name": results_root.name,
        "rootfs": str(rootfs),
        "build_dir": str(build_dir),
        "created_at": time.time(),
        "task_count": len(tasks),
        "controllers": sorted({task["controller"] for task in tasks}),
        "traj_ids": sorted({int(task["traj_id"]) for task in tasks}),
        "sim_speeds": sorted({float(task["sim_speed_factor"]) for task in tasks}),
        "modes": sorted({task["mode"] for task in tasks}),
    }
    (results_root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rows: List[Dict] = []
    for idx, task in enumerate(tasks, start=1):
        case_key = f"{task['controller']}_traj{task['traj_id']}_{task['param_set_name']}_{task['mode']}_x{task['sim_speed_factor']}_r{task['repeat_index']}"
        run_dir = results_root / case_key
        print(f"[{idx}/{len(tasks)}] {case_key}", flush=True)
        row = run_case(
            tool_root,
            repo_root,
            rootfs,
            build_dir,
            run_dir,
            controller=task["controller"],
            traj_id=int(task["traj_id"]),
            params=task["params"],
            sim_speed_factor=float(task["sim_speed_factor"]),
            mode=str(task["mode"]),
            repeat_index=int(task["repeat_index"]),
            takeoff_alt=float(args.takeoff_alt),
            takeoff_timeout=float(args.takeoff_timeout),
            trajectory_timeout=float(args.trajectory_timeout),
            landing_mode=str(args.landing_mode),
            engagement_dwell_s=float(args.engagement_dwell_s),
        )
        row["param_set_name"] = task["param_set_name"]
        row["params"] = task["params"]
        rows.append(row)
        (results_root / "rows.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")

    with (results_root / "rows.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "controller", "traj_id", "param_set_name", "mode", "sim_speed_factor", "repeat_index",
            "returncode", "wall_s", "state_status", "display_mode",
            "fitness", "track_rmse", "energy_term", "tracking_log_hash", "tracking_log_copy",
            "result_json", "state_json", "stdout_log", "error",
        ])
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in writer.fieldnames})

    grouped: Dict[str, List[Dict]] = defaultdict(list)
    for row in rows:
        key = f"{row['controller']}|{row['traj_id']}|{row['param_set_name']}|{row['mode']}|{row['sim_speed_factor']}"
        grouped[key].append(row)
    summaries = {key: summarize_group(group) for key, group in grouped.items()}

    baseline_rows: Dict[str, List[Dict]] = defaultdict(list)
    for row in rows:
        if math.isclose(float(row["sim_speed_factor"]), 1.0, rel_tol=0.0, abs_tol=1e-9):
            key = f"{row['controller']}|{row['traj_id']}|{row['param_set_name']}|{row['mode']}"
            baseline_rows[key].append(row)

    speed_comparison: Dict[str, Dict] = {}
    for key, group in grouped.items():
        controller, traj_id, param_set_name, mode, sim_speed = key.split("|")
        base_key = f"{controller}|{traj_id}|{param_set_name}|{mode}"
        speed_comparison[key] = compare_to_baseline(group, baseline_rows.get(base_key, []))

    overall = {
        "all_runs_ok": all(int(row.get("returncode", 1)) == 0 and str(row.get("state_status")) == "finished" for row in rows),
        "worst_fitness_spread": max((summary["metrics"].get("fitness", {}).get("spread", 0.0) for summary in summaries.values()), default=0.0),
        "worst_track_spread": max((summary["metrics"].get("track_rmse", {}).get("spread", 0.0) for summary in summaries.values()), default=0.0),
        "worst_energy_spread": max((summary["metrics"].get("energy_term", {}).get("spread", 0.0) for summary in summaries.values()), default=0.0),
        "max_hash_count": max((summary.get("hash_count", 0) for summary in summaries.values()), default=0),
    }
    summary_payload = {
        "manifest": manifest,
        "overall": overall,
        "groups": summaries,
        "speed_comparison": speed_comparison,
    }
    (results_root / "summary.json").write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    print(json.dumps(summary_payload["overall"], indent=2))
    return 0 if overall["all_runs_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
