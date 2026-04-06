#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.offnominal_sitl_study import run_identification_with_assets


def main() -> int:
    ap = argparse.ArgumentParser(description="Run the nine-profile SITL identification suite for a prepared Gazebo model.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--out-root", required=True)
    ap.add_argument("--label", required=True, help="Output label under the chosen out-root.")
    ap.add_argument("--model-name", required=True, help="Gazebo model name that PX4 should spawn.")
    ap.add_argument("--source-model-dir", required=True, help="Prepared Gazebo model directory to copy into the run override.")
    ap.add_argument("--visual", action="store_true", help="Run Gazebo with GUI instead of headless mode.")
    ap.add_argument("--show-console", action="store_true", help="Open the read-only PX4 log window in visual mode.")
    args = ap.parse_args()

    manifest = run_identification_with_assets(
        px4_root=Path(args.px4_root).expanduser().resolve(),
        out_root=Path(args.out_root).expanduser().resolve(),
        label=args.label,
        source_model_dir=Path(args.source_model_dir).expanduser().resolve(),
        model_name=args.model_name,
        headless=not args.visual,
        show_console=args.show_console,
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
