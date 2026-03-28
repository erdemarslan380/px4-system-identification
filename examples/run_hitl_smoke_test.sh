#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${1:-/dev/ttyUSB0}"
BAUDRATE="${2:-57600}"

cd "$REPO_ROOT"
python3 examples/hitl_shell_smoke.py \
  --port "$DEVICE" \
  --baudrate "$BAUDRATE" \
  --cmd 'pwm_out_sim status' \
  --cmd 'free' \
  --cmd 'custom_pos_control start' \
  --cmd 'trajectory_reader start' \
  --cmd 'custom_pos_control enable' \
  --cmd 'custom_pos_control set px4_default' \
  --cmd 'trajectory_reader set_mode position' \
  --cmd 'trajectory_reader abs_ref 0 0 -3 0' \
  --cmd 'trajectory_reader set_traj_anchor 0 0 -3' \
  --cmd 'trajectory_reader set_traj_id 100' \
  --cmd 'trajectory_reader status' \
  --require 'pwm_out_sim:' \
  --require 'Umem:' \
  --require 'custom_pos_control enabled' \
  --require 'TrajectoryReader constructed' \
  --require 'Trajectory ID set to 100'
