#!/usr/bin/env python3
"""Run the full overnight sequence: identification -> compare -> validation -> broad run."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml

from plan_runtime import is_pid_alive, load_json, normalize_manifest_runtime, runtime_state_path, write_json
from plan_watchdog import launch_detached


def plan_results_root(repo_root: Path, plan_path: Path) -> Path:
    raw_plan = yaml.safe_load(plan_path.read_text(encoding="utf-8")) or {}
    raw_results_root = str(raw_plan.get("results_root", f"Tools/optimization/plan_runs/{plan_path.stem}"))
    path = Path(raw_results_root)
    return (repo_root / path).resolve() if not path.is_absolute() else path.resolve()


def manifest_path_for_plan(repo_root: Path, plan_path: Path) -> Path:
    return plan_results_root(repo_root, plan_path) / "plan_manifest.json"


def manifest_done_ok(manifest: dict | None) -> bool:
    if not manifest:
        return False
    counts = manifest.get("counts") or {}
    total = len(manifest.get("tasks", []))
    return total > 0 and int(counts.get("ok", 0)) == total and int(counts.get("pending", 0)) == 0 and int(counts.get("running", 0)) == 0 and int(counts.get("failed", 0)) == 0


def manifest_has_unfinished(manifest: dict | None) -> bool:
    if not manifest:
        return False
    counts = manifest.get("counts") or {}
    return any(int(counts.get(key, 0)) > 0 for key in ("pending", "running", "failed"))


def normalize_gz_plan_defaults(plan_path: Path) -> None:
    raw_plan = yaml.safe_load(plan_path.read_text(encoding="utf-8")) or {}
    defaults = raw_plan.setdefault("defaults", {})
    simulator = str(defaults.get("simulator", "") or "").strip().lower()
    if simulator != "gz":
        return

    changed = False
    if str(defaults.get("workers", "")).strip() != "1":
        defaults["workers"] = 1
        changed = True
    if float(defaults.get("takeoff_timeout", 0.0) or 0.0) < 45.0:
        defaults["takeoff_timeout"] = 45.0
        changed = True

    if changed:
        plan_path.write_text(yaml.safe_dump(raw_plan, sort_keys=False), encoding="utf-8")


def wait_for_plan(repo_root: Path, tool_root: Path, plan_path: Path, *, clean_start: bool, serve_dashboard: bool, max_resumes: int) -> dict:
    existing_manifest_path = manifest_path_for_plan(repo_root, plan_path)
    existing_results_root = plan_results_root(repo_root, plan_path)
    existing_manifest = load_json(existing_manifest_path)
    existing_runtime = load_json(runtime_state_path(existing_results_root))
    existing_runner_alive = is_pid_alive((existing_runtime or {}).get("pid"), cmdline_contains="run_simulation_plan.py")

    state = {
        "plan": str(plan_path),
        "results_root": str(existing_results_root),
        "resume_count": 0,
        "started_at": time.time(),
        "status": "starting",
    }
    state_path = tool_root / "results" / f"{plan_path.stem}_sequence_state.json"
    write_json(state_path, state)

    if existing_manifest and manifest_done_ok(existing_manifest):
        state["status"] = "finished"
        state["last_launch"] = {
            "manifest": str(existing_manifest_path.resolve()),
            "results_root": str(existing_results_root.resolve()),
            "attached": True,
        }
        state["finished_at"] = time.time()
        write_json(state_path, state)
        return existing_manifest

    launch = None
    if existing_manifest and existing_runner_alive:
        manifest_path = existing_manifest_path.resolve()
        results_root = existing_results_root.resolve()
        state["status"] = "attached"
        state["last_launch"] = {
            "manifest": str(manifest_path),
            "results_root": str(results_root),
            "attached": True,
        }
    else:
        launch = launch_detached(repo_root, tool_root, plan_path, clean=clean_start, serve_dashboard=serve_dashboard, resume=False)
        manifest_path = Path(launch["manifest"]).resolve()
        results_root = Path(launch["results_root"]).resolve()
        state["status"] = "running"
        state["last_launch"] = launch
    write_json(state_path, state)

    while True:
        time.sleep(20.0)
        manifest = load_json(manifest_path)
        runtime = load_json(runtime_state_path(results_root))
        if manifest:
            normalized, changed = normalize_manifest_runtime(manifest)
            if changed:
                manifest = normalized
                write_json(manifest_path, manifest)
        if manifest_done_ok(manifest):
            state["status"] = "finished"
            state["finished_at"] = time.time()
            write_json(state_path, state)
            return manifest

        runner_alive = is_pid_alive((runtime or {}).get("pid"), cmdline_contains="run_simulation_plan.py")
        state["runner_alive"] = runner_alive
        state["last_counts"] = (manifest or {}).get("counts", {})
        state["updated_at"] = time.time()
        write_json(state_path, state)

        if runner_alive:
            continue

        if not manifest_has_unfinished(manifest):
            state["status"] = "finished"
            state["finished_at"] = time.time()
            write_json(state_path, state)
            return manifest or {}

        if state["resume_count"] >= max_resumes:
            state["status"] = "failed"
            state["finished_at"] = time.time()
            write_json(state_path, state)
            raise RuntimeError(f"{plan_path.name}: exceeded max resumes ({max_resumes})")

        launch = launch_detached(repo_root, tool_root, plan_path, clean=False, serve_dashboard=serve_dashboard, resume=True)
        manifest_path = Path(launch["manifest"]).resolve()
        results_root = Path(launch["results_root"]).resolve()
        state["resume_count"] += 1
        state["last_launch"] = launch
        state["status"] = "running"
        write_json(state_path, state)


def start_watchdog_detached(repo_root: Path, tool_root: Path, plan_path: Path, *, serve_dashboard: bool) -> dict:
    log_dir = tool_root / "results" / "watchdog_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{plan_path.stem}.log"
    cmd = [
        sys.executable,
        str(tool_root / "plan_watchdog.py"),
        "--plan",
        str(plan_path),
        "--clean-start",
    ]
    if serve_dashboard:
        cmd.append("--serve-dashboard")
    proc = subprocess.Popen(
        cmd,
        cwd=str(repo_root),
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return {"pid": proc.pid, "log": str(log_path)}


def main() -> int:
    ap = argparse.ArgumentParser(description="Queue the nightly Gazebo pipeline.")
    ap.add_argument("--ident-plan", default="Tools/optimization/generated_plans/x500_identification_suite.yaml")
    ap.add_argument("--quick-plan", default="Tools/optimization/generated_plans/all_controllers_quick_validation.yaml")
    ap.add_argument("--overnight-plan", default="Tools/optimization/generated_plans/overnight_bayes_viable.yaml")
    ap.add_argument("--compare-out", default="experimental_validation/outputs/x500_identification_suite")
    ap.add_argument("--serve-dashboard", action="store_true")
    ap.add_argument("--max-resumes", type=int, default=6)
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    ident_plan = (repo_root / args.ident_plan).resolve() if not Path(args.ident_plan).is_absolute() else Path(args.ident_plan).resolve()
    quick_plan = (repo_root / args.quick_plan).resolve() if not Path(args.quick_plan).is_absolute() else Path(args.quick_plan).resolve()
    overnight_plan = (repo_root / args.overnight_plan).resolve() if not Path(args.overnight_plan).is_absolute() else Path(args.overnight_plan).resolve()
    compare_out = (repo_root / args.compare_out).resolve() if not Path(args.compare_out).is_absolute() else Path(args.compare_out).resolve()

    normalize_gz_plan_defaults(ident_plan)
    normalize_gz_plan_defaults(quick_plan)
    normalize_gz_plan_defaults(overnight_plan)

    pipeline_state = tool_root / "results" / "nightly_pipeline_state.json"
    write_json(pipeline_state, {
        "status": "identification",
        "ident_plan": str(ident_plan),
        "quick_plan": str(quick_plan),
        "overnight_plan": str(overnight_plan),
        "started_at": time.time(),
    })

    ident_manifest = wait_for_plan(
        repo_root,
        tool_root,
        ident_plan,
        clean_start=not manifest_path_for_plan(repo_root, ident_plan).exists(),
        serve_dashboard=args.serve_dashboard,
        max_resumes=args.max_resumes,
    )

    compare_cmd = [
        sys.executable,
        str(repo_root / "experimental_validation" / "compare_with_sdf.py"),
        "--results-root",
        str(plan_results_root(repo_root, ident_plan)),
        "--out-dir",
        str(compare_out),
    ]
    compare_proc = subprocess.run(compare_cmd, cwd=str(repo_root), capture_output=True, text=True, timeout=60.0)
    if compare_proc.returncode != 0:
        raise RuntimeError(compare_proc.stderr.strip() or compare_proc.stdout.strip() or "SDF comparison failed")

    write_json(pipeline_state, {
        "status": "quick_validation",
        "ident_manifest": str(manifest_path_for_plan(repo_root, ident_plan)),
        "compare_out": str(compare_out),
        "quick_plan": str(quick_plan),
        "overnight_plan": str(overnight_plan),
        "updated_at": time.time(),
    })

    wait_for_plan(
        repo_root,
        tool_root,
        quick_plan,
        clean_start=True,
        serve_dashboard=args.serve_dashboard,
        max_resumes=args.max_resumes,
    )

    watchdog = start_watchdog_detached(repo_root, tool_root, overnight_plan, serve_dashboard=args.serve_dashboard)
    write_json(pipeline_state, {
        "status": "overnight_running",
        "compare_out": str(compare_out),
        "overnight_plan": str(overnight_plan),
        "watchdog": watchdog,
        "updated_at": time.time(),
    })
    print(json.dumps({
        "ok": True,
        "compare_out": str(compare_out),
        "watchdog": watchdog,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
