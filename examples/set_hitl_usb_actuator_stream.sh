#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RATE_HZ="${1:-200}"
ENDPOINT="${2:-${PX4_HIL_ENDPOINT:-udpin:127.0.0.1:14550}}"
CONTROL_PORT="${PX4_HIL_CTRL_PTY:-/tmp/px4_hitl_ctrl.pty}"

if [[ -n "$ENDPOINT" ]]; then
  NSH_PORT="$ENDPOINT"
  NSH_BAUD=57600
elif [[ -e "$CONTROL_PORT" ]]; then
  NSH_PORT="$CONTROL_PORT"
  NSH_BAUD=921600
elif [[ -e /dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00 ]]; then
  NSH_PORT="/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00"
  NSH_BAUD=57600
elif [[ -e /dev/ttyACM0 ]]; then
  NSH_PORT="/dev/ttyACM0"
  NSH_BAUD=57600
else
  NSH_PORT="$(ls -1 /dev/ttyACM* 2>/dev/null | tail -n 1)"
  NSH_BAUD=57600
fi

if [[ -z "${NSH_PORT:-}" ]]; then
  echo "No NSH control port found." >&2
  exit 2
fi

python3 "$SCRIPT_DIR/px4_nsh_runner.py" \
  --port "$NSH_PORT" \
  --baud "$NSH_BAUD" \
  --cmd "mavlink stream -d /dev/ttyACM0 -s HIL_ACTUATOR_CONTROLS -r $RATE_HZ"
