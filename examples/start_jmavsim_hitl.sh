#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
DEVICE="${2:-/dev/ttyACM0}"
BAUDRATE="${3:-921600}"

if [[ "${PX4_SYSID_HEADLESS:-0}" == "1" ]]; then
  export HEADLESS=1
fi

cd "$PX4_ROOT"
./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$DEVICE" -b "$BAUDRATE" -r 250
