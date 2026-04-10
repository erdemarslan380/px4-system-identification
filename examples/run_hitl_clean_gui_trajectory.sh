#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
TRAJ_ID="${2:-100}"
ENDPOINT="${3:-udpin:127.0.0.1:14550}"
BAUDRATE="${4:-921600}"

case "$TRAJ_ID" in
  100|101|102|103|104) ;;
  *)
    echo "Unsupported trajectory id: $TRAJ_ID (expected 100..104)" >&2
    exit 2
    ;;
esac

export PX4_HIL_SIM_LOOP_RATE_HZ="${PX4_HIL_SIM_LOOP_RATE_HZ:-1000}"
export PX4_HIL_USB_STREAM_RATE_HZ="${PX4_HIL_USB_STREAM_RATE_HZ:-200}"
export PX4_HIL_IMU_INTEG_RATE_HZ="${PX4_HIL_IMU_INTEG_RATE_HZ:-250}"
export PX4_HIL_RATE_LOOP_HZ="${PX4_HIL_RATE_LOOP_HZ:-1000}"
export PX4_HIL_ATTITUDE_LOOP_HZ="${PX4_HIL_ATTITUDE_LOOP_HZ:-250}"
export PX4_HIL_POSITION_LOOP_HZ="${PX4_HIL_POSITION_LOOP_HZ:-100}"

HOVER_Z="${PX4_HIL_TRAJ_HOVER_Z:--5.0}"
TRAJ_Z_TOLERANCE="${PX4_HIL_TRAJ_Z_TOLERANCE:-0.5}"
TRAJ_TILT_LIMIT_DEG="${PX4_HIL_TRAJ_TILT_LIMIT_DEG:-35}"
TRAJ_XY_ENVELOP_LIMIT="${PX4_HIL_TRAJ_XY_ENVELOP_LIMIT:-1.5}"

"$SCRIPT_DIR/restart_hitl_px4_clean_gui.sh" "$PX4_ROOT" "$BAUDRATE" "$ENDPOINT"

python3 -u "$SCRIPT_DIR/run_hitl_px4_module_position_hold.py" \
  --endpoint "$ENDPOINT" \
  --hover-z "$HOVER_Z" \
  --hold-seconds 5 \
  --settle-seconds 1.5 \
  --sysid-hold-seconds 0 \
  --traj-id "$TRAJ_ID" \
  --tilt-limit-deg 10 \
  --trajectory-xy-envelop-limit "$TRAJ_XY_ENVELOP_LIMIT" \
  --trajectory-z-tolerance "$TRAJ_Z_TOLERANCE" \
  --trajectory-tail-seconds 10 \
  --trajectory-tilt-limit-deg "$TRAJ_TILT_LIMIT_DEG"
