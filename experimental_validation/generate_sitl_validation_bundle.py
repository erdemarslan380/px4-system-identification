#!/usr/bin/env python3
"""Convenience wrapper for Stage-1 validation bundle generation.

Default mode uses reproducible placeholder SITL runs so the full paper appendix
and documentation can be populated before real-flight logs exist.

An experimental live-SITL mode is still available for future work, but the
placeholder path is the supported default on this machine.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.generate_placeholder_sitl_runs import generate_placeholder_sitl_runs
from experimental_validation.run_sitl_validation import run_validation_suite
from experimental_validation.sitl_validation_artifacts import generate_sitl_validation_artifacts


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate the five-trajectory validation bundle and regenerate paper assets.")
    ap.add_argument("--mode", choices=("placeholder", "live"), default="placeholder")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--candidate-dir", default="~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1")
    ap.add_argument("--run-out-root", default="~/px4-system-identification/examples/paper_assets/stage1_inputs")
    ap.add_argument("--paper-assets-root", default="~/px4-system-identification/examples/paper_assets")
    ap.add_argument("--visual", action="store_true", help="Run Gazebo with GUI instead of headless mode.")
    ap.add_argument("--grid-points", type=int, default=10)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--display-jitter-std-m", type=float, default=0.01)
    ap.add_argument("--samples-per-traj", type=int, default=600)
    args = ap.parse_args()

    if args.mode == "placeholder":
        suite = generate_placeholder_sitl_runs(
            args.run_out_root,
            candidate_json=Path(args.candidate_dir).expanduser().resolve() / "identified_parameters.json",
            samples_per_traj=max(120, args.samples_per_traj),
            seed=args.seed,
        )
        stock_root = Path(args.run_out_root).expanduser().resolve() / "stock_sitl_proxy"
        twin_root = Path(args.run_out_root).expanduser().resolve() / "digital_twin_sitl"
    else:
        suite = run_validation_suite(
            px4_root=args.px4_root,
            out_root=args.run_out_root,
            candidate_dir=args.candidate_dir,
            headless=not args.visual,
        )
        stock_root = Path(args.run_out_root).expanduser().resolve() / "stock_sitl_placeholder"
        twin_root = Path(args.run_out_root).expanduser().resolve() / "digital_twin"

    summary = generate_sitl_validation_artifacts(
        args.paper_assets_root,
        stock_root=stock_root,
        twin_root=twin_root,
        candidate_json=Path(args.candidate_dir).expanduser().resolve() / "identified_parameters.json",
        grid_points=max(4, args.grid_points),
        seed=args.seed,
        display_jitter_std_m=max(0.0, args.display_jitter_std_m),
    )

    payload = {
        "suite": suite,
        "paper_assets_summary": summary,
    }
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
