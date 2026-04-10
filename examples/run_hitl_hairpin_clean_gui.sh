#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
ENDPOINT="${2:-udpin:127.0.0.1:14550}"
BAUDRATE="${3:-921600}"

"$SCRIPT_DIR/run_hitl_clean_gui_trajectory.sh" "$PX4_ROOT" 100 "$ENDPOINT" "$BAUDRATE"
