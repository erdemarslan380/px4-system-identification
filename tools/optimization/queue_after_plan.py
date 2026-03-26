#!/usr/bin/env python3
"""Wait for one plan to finish, then launch the next plan under the watchdog."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from plan_runtime import write_json
from queue_nightly_pipeline import start_watchdog_detached, wait_for_plan


def wait_then_start(repo_root: Path,
                    tool_root: Path,
                    wait_plan: Path,
                    next_plan: Path,
                    *,
                    serve_dashboard: bool,
                    max_resumes: int) -> dict:
    state_path = tool_root / "results" / f"{wait_plan.stem}_to_{next_plan.stem}_queue.json"
    state = {
        "status": "waiting",
        "wait_plan": str(wait_plan),
        "next_plan": str(next_plan),
        "started_at": time.time(),
    }
    write_json(state_path, state)

    wait_manifest = wait_for_plan(
        repo_root,
        tool_root,
        wait_plan,
        clean_start=False,
        serve_dashboard=serve_dashboard,
        max_resumes=max_resumes,
    )

    watchdog = start_watchdog_detached(repo_root, tool_root, next_plan, serve_dashboard=serve_dashboard)
    state.update({
        "status": "next_started",
        "wait_manifest": str(wait_manifest.get("results_root", "")),
        "watchdog": watchdog,
        "finished_at": time.time(),
    })
    write_json(state_path, state)
    return {
        "ok": True,
        "watchdog": watchdog,
        "state_path": str(state_path),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Wait for one plan to finish, then launch another.")
    ap.add_argument("--wait-plan", required=True)
    ap.add_argument("--next-plan", required=True)
    ap.add_argument("--serve-dashboard", action="store_true")
    ap.add_argument("--max-resumes", type=int, default=8)
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    wait_plan = (repo_root / args.wait_plan).resolve() if not Path(args.wait_plan).is_absolute() else Path(args.wait_plan).resolve()
    next_plan = (repo_root / args.next_plan).resolve() if not Path(args.next_plan).is_absolute() else Path(args.next_plan).resolve()

    result = wait_then_start(
        repo_root,
        tool_root,
        wait_plan,
        next_plan,
        serve_dashboard=args.serve_dashboard,
        max_resumes=args.max_resumes,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
