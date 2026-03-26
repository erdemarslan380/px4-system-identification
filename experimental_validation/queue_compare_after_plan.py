#!/usr/bin/env python3
"""Wait for an identification plan to finish, then generate SDF comparison outputs."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_ROOT = REPO_ROOT / "Tools" / "optimization"

if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import (
    PRIMARY_IDENTIFICATION_MODE,
    build_identification_mode_reports,
    collect_identification_logs,
    parse_x500_sdf_reference,
    write_comparison_outputs,
)
from plan_runtime import is_pid_alive, load_json, runtime_state_path, write_json
from queue_nightly_pipeline import manifest_done_ok, manifest_path_for_plan, plan_results_root, wait_for_plan


def wait_for_existing_plan_completion(
    *,
    wait_plan: Path,
    serve_dashboard: bool,
    max_resumes: int,
    poll_s: float = 10.0,
) -> dict:
    manifest_path = manifest_path_for_plan(REPO_ROOT, wait_plan)
    results_root = plan_results_root(REPO_ROOT, wait_plan)

    while True:
        manifest = load_json(manifest_path)
        runtime = load_json(runtime_state_path(results_root))
        runner_alive = is_pid_alive((runtime or {}).get("pid"), cmdline_contains="run_simulation_plan.py")
        if runner_alive or manifest_done_ok(manifest):
            return wait_for_plan(
                REPO_ROOT,
                TOOLS_ROOT,
                wait_plan,
                clean_start=False,
                serve_dashboard=serve_dashboard,
                max_resumes=max_resumes,
            )
        time.sleep(poll_s)


def wait_then_compare(
    *,
    wait_plan: Path,
    out_dir: Path,
    sdf_model: Path,
    sdf_base_model: Path,
    serve_dashboard: bool,
    max_resumes: int,
) -> dict:
    state_path = REPO_ROOT / "experimental_validation" / "outputs" / f"{wait_plan.stem}_compare_queue.json"
    state = {
        "status": "waiting",
        "wait_plan": str(wait_plan),
        "out_dir": str(out_dir),
        "started_at": time.time(),
    }
    write_json(state_path, state)

    manifest = wait_for_existing_plan_completion(
        wait_plan=wait_plan,
        serve_dashboard=serve_dashboard,
        max_resumes=max_resumes,
    )
    results_root = Path(str(manifest.get("results_root") or "")).resolve()
    logs = collect_identification_logs(results_root)
    sdf_reference = parse_x500_sdf_reference(sdf_model, sdf_base_model)
    reports_by_mode = build_identification_mode_reports(logs, sdf_reference)
    write_comparison_outputs(
        out_dir.resolve(),
        reports_by_mode=reports_by_mode,
        sdf_reference=sdf_reference,
        csv_paths=logs,
        primary_mode=PRIMARY_IDENTIFICATION_MODE,
    )

    state.update({
        "status": "done",
        "results_root": str(results_root),
        "finished_at": time.time(),
    })
    write_json(state_path, state)
    return {
        "ok": True,
        "results_root": str(results_root),
        "out_dir": str(out_dir.resolve()),
        "state_path": str(state_path),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Wait for an identification plan, then run SDF comparison.")
    ap.add_argument("--wait-plan", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--sdf-model", default="Tools/simulation/gz/models/x500/model.sdf")
    ap.add_argument("--sdf-base-model", default="Tools/simulation/gz/models/x500_base/model.sdf")
    ap.add_argument("--serve-dashboard", action="store_true")
    ap.add_argument("--max-resumes", type=int, default=8)
    args = ap.parse_args()

    wait_plan = (REPO_ROOT / args.wait_plan).resolve() if not Path(args.wait_plan).is_absolute() else Path(args.wait_plan).resolve()
    out_dir = (REPO_ROOT / args.out_dir).resolve() if not Path(args.out_dir).is_absolute() else Path(args.out_dir).resolve()
    sdf_model = (REPO_ROOT / args.sdf_model).resolve() if not Path(args.sdf_model).is_absolute() else Path(args.sdf_model).resolve()
    sdf_base_model = (REPO_ROOT / args.sdf_base_model).resolve() if not Path(args.sdf_base_model).is_absolute() else Path(args.sdf_base_model).resolve()

    result = wait_then_compare(
        wait_plan=wait_plan,
        out_dir=out_dir,
        sdf_model=sdf_model,
        sdf_base_model=sdf_base_model,
        serve_dashboard=args.serve_dashboard,
        max_resumes=args.max_resumes,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
