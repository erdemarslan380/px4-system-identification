#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
DEVICE="${2:-/dev/ttyACM0}"
BAUDRATE="${3:-921600}"

if [[ "${PX4_SYSID_HEADLESS:-0}" == "1" ]]; then
  export HEADLESS=1
fi

if [[ ! -e "$DEVICE" ]]; then
  if [[ $# -lt 2 ]]; then
    shopt -s nullglob
    acm_candidates=(/dev/ttyACM*)
    shopt -u nullglob
    if [[ ${#acm_candidates[@]} -eq 1 ]]; then
      echo "Default $DEVICE is not present; using ${acm_candidates[0]} instead." >&2
      DEVICE="${acm_candidates[0]}"
    fi
  fi
fi

if [[ ! -e "$DEVICE" ]]; then
  echo "Serial device $DEVICE does not exist." >&2
  echo "Available serial devices:" >&2
  ls -1 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null >&2 || echo "(none)" >&2
  exit 2
fi

if command -v lsof >/dev/null 2>&1; then
  HOLDER_PIDS="$(lsof -t "$DEVICE" 2>/dev/null | tr '\n' ' ' || true)"
  if [[ -n "${HOLDER_PIDS// }" ]]; then
    echo "Serial device $DEVICE is already open: $HOLDER_PIDS" >&2
    echo "Close QGroundControl, mavlink_shell, or any other process using $DEVICE before starting jMAVSim." >&2
    echo "Available serial devices:" >&2
    ls -1 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null >&2 || echo "(none)" >&2
    exit 2
  fi
fi

cd "$PX4_ROOT"
./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$DEVICE" -b "$BAUDRATE" -r 250
