#!/usr/bin/env python3
"""Build a truth-assisted SITL candidate and refresh paper assets from one results root."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _run(cmd: list[str]) -> None:
    proc = subprocess.run(cmd, cwd=str(REPO_ROOT), text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed: {' '.join(cmd)}\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}")


def refresh_truth_artifacts(results_root: str | Path, *, candidate_name: str, out_dir: str | Path) -> dict:
    results_root = Path(results_root).resolve()
    out_dir = Path(out_dir).resolve()
    candidate_dir = out_dir / "candidates" / candidate_name
    candidate_dir.mkdir(parents=True, exist_ok=True)

    compare_cmd = [
        sys.executable,
        str(REPO_ROOT / "experimental_validation" / "compare_with_sdf.py"),
        "--results-root",
        str(results_root),
        "--out-dir",
        str(candidate_dir),
    ]
    _run(compare_cmd)

    candidate_json = candidate_dir / "identified_parameters.json"
    figures_cmd = [
        sys.executable,
        str(REPO_ROOT / "experimental_validation" / "paper_artifacts.py"),
        "--out-dir",
        str(out_dir),
        "--candidate-json",
        str(candidate_json),
    ]
    _run(figures_cmd)

    summary = json.loads((out_dir / "paper_validation_summary.json").read_text(encoding="utf-8"))
    return {
        "candidate_dir": str(candidate_dir),
        "candidate_json": str(candidate_json),
        "summary_path": str(out_dir / "paper_validation_summary.json"),
        "base_blended_twin_score": float(summary["base_blended_twin_score"]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Refresh truth-assisted SITL candidate outputs and paper figures from a finished results root.")
    ap.add_argument("--results-root", required=True)
    ap.add_argument("--candidate-name", default="x500_truth_assisted_sitl_v1")
    ap.add_argument("--out-dir", default="examples/paper_assets")
    args = ap.parse_args()

    payload = refresh_truth_artifacts(
        args.results_root,
        candidate_name=args.candidate_name,
        out_dir=(REPO_ROOT / args.out_dir) if not Path(args.out_dir).is_absolute() else Path(args.out_dir),
    )
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
