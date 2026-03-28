#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_TREE="${1:-$HOME/PX4-Autopilot-Identification}"
TRAJ_DIR="$PX4_TREE/build/px4_sitl_default/rootfs/trajectories"
ASSETS_ROOT="$REPO_ROOT/examples/paper_assets"
CANDIDATE_DIR="$ASSETS_ROOT/candidates/x500_truth_assisted_sitl_v1"
HITL_REVIEW_DEMO_DIR="$REPO_ROOT/examples/hitl_review_demo"

mkdir -p "$TRAJ_DIR"

cd "$REPO_ROOT"
python3 experimental_validation/validation_trajectories.py \
  --trajectories-dir "$TRAJ_DIR"

python3 experimental_validation/generate_sitl_validation_bundle.py \
  --mode placeholder \
  --candidate-dir "$CANDIDATE_DIR" \
  --run-out-root "$ASSETS_ROOT/stage1_inputs" \
  --paper-assets-root "$ASSETS_ROOT"

python3 experimental_validation/build_hitl_review_bundle.py \
  --log-root "$ASSETS_ROOT/stage1_inputs/digital_twin_sitl" \
  --out-dir "$HITL_REVIEW_DEMO_DIR"

printf '\nDemo assets refreshed successfully.\n'
printf 'Trajectories: %s\n' "$TRAJ_DIR"
printf 'Figures: %s\n' "$ASSETS_ROOT/figures"
printf 'Summary: %s\n' "$ASSETS_ROOT/paper_validation_summary.json"
printf 'Review UI: %s\n' "$HITL_REVIEW_DEMO_DIR/index.html"
