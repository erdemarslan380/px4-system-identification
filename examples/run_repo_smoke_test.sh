#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

python3 -m unittest \
  experimental_validation.tests.test_repo_cleanliness \
  experimental_validation.tests.test_px4_hardware_helpers \
  experimental_validation.tests.test_hitl_review_bundle \
  experimental_validation.tests.test_pull_sdcard_logs_over_mavftp \
  experimental_validation.tests.test_validation_trajectories \
  experimental_validation.tests.test_generate_placeholder_sitl_runs \
  experimental_validation.tests.test_sitl_validation_artifacts \
  experimental_validation.tests.test_paper_artifacts
