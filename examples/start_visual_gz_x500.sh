#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 /path/to/PX4-Autopilot"
  exit 1
fi

PX4_ROOT="$1"
cd "$PX4_ROOT"
unset HEADLESS
make px4_sitl gz_x500
