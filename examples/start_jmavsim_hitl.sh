#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
DEVICE="${2:-/dev/ttyACM0}"
BAUDRATE="${3:-921600}"

if [[ "${PX4_SYSID_HEADLESS:-0}" == "1" ]]; then
  export HEADLESS=1
fi

if command -v lsof >/dev/null 2>&1; then
  HOLDER_PIDS="$(lsof -t "$DEVICE" 2>/dev/null | tr '\n' ' ')"
  if [[ -n "${HOLDER_PIDS// }" ]]; then
    echo "Serial device $DEVICE is already open: $HOLDER_PIDS" >&2
    echo "Close QGroundControl, mavlink_shell, or any other process using $DEVICE before starting jMAVSim." >&2
    exit 2
  fi
fi

cd "$PX4_ROOT"
./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$DEVICE" -b "$BAUDRATE" -r 250
