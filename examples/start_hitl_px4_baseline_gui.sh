#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
DEVICE="${2:-}"
BAUDRATE="${3:-921600}"
ENDPOINT="${PX4_HIL_STOP_ENDPOINT:-udpin:127.0.0.1:14550}"

graceful_stop_pattern() {
  local label="$1"
  local pattern="$2"
  local -a pids=()
  mapfile -t pids < <(pgrep -f "$pattern" || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Requesting graceful stop for $label: ${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null || true

  for _ in $(seq 1 40); do
    local alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    if [[ $alive -eq 0 ]]; then
      return 0
    fi
    sleep 0.25
  done

  echo "$label did not exit cleanly; refusing to start a new HIL GUI session." >&2
  return 1
}

graceful_close_jmavsim_gui() {
  local -a win_ids=()
  local -a pids=()
  mapfile -t pids < <(pgrep -f 'java.*jmavsim_run.jar|jmavsim_run.sh -q -s -d ' || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    return 0
  fi

  if command -v wmctrl >/dev/null 2>&1; then
    mapfile -t win_ids < <(wmctrl -lx 2>/dev/null | awk 'tolower($0) ~ /jmavsim/ {print $1}')
    if [[ ${#win_ids[@]} -gt 0 ]]; then
      echo "Closing jMAVSim window via wmctrl: ${win_ids[*]}"
      for wid in "${win_ids[@]}"; do
        wmctrl -ic "$wid" || true
      done
    fi
  fi

  if [[ ${#win_ids[@]} -eq 0 ]] && command -v xdotool >/dev/null 2>&1; then
    mapfile -t win_ids < <(xdotool search --name '^jMAVSim$' 2>/dev/null || true)
    if [[ ${#win_ids[@]} -gt 0 ]]; then
      echo "Closing jMAVSim window via xdotool: ${win_ids[*]}"
      for wid in "${win_ids[@]}"; do
        xdotool windowclose "$wid" || true
      done
    fi
  fi

  if [[ ${#win_ids[@]} -eq 0 ]]; then
    echo "jMAVSim is running but no GUI window was found; refusing abrupt shutdown." >&2
    return 1
  fi

  for _ in $(seq 1 80); do
    local alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    if [[ $alive -eq 0 ]]; then
      return 0
    fi
    sleep 0.25
  done

  echo "jMAVSim window close did not shut down the process cleanly." >&2
  return 1
}

graceful_stop_existing_hil() {
  graceful_stop_pattern "HIL helpers" 'run_hitl_px4_|run_hitl_full_stack_' || return 1
  graceful_close_jmavsim_gui || return 1
  graceful_stop_pattern "mavlink serial hub" 'mavlink_serial_hub.py' || return 1
}

if [[ -z "$DEVICE" ]]; then
  shopt -s nullglob
  cube_candidates=(/dev/serial/by-id/*CubeOrange*)
  shopt -u nullglob
  if [[ ${#cube_candidates[@]} -ge 1 ]]; then
    DEVICE="${cube_candidates[0]}"
  else
    DEVICE="/dev/ttyACM0"
  fi
fi

echo "Starting clean baseline HIL GUI session"
echo "PX4 root : $PX4_ROOT"
echo "Device   : $DEVICE"
echo "Baud     : $BAUDRATE"
echo "Stop EP  : $ENDPOINT"

python3 - "$ENDPOINT" <<'PY' || true
import sys
import time
from pymavlink import mavutil

endpoint = sys.argv[1]
try:
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=2)
    if hb:
        for name, value, param_type in (
            ("TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ):
            mav.mav.param_set_send(
                mav.target_system,
                mav.target_component,
                name.encode("ascii"),
                float(value),
                param_type,
            )
            time.sleep(0.1)

        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0, 0, 0, 0, 0,
        )
        time.sleep(0.5)
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        print("Sent PX4 safe shutdown before replacing existing HIL GUI session")
except Exception as exc:
    print(f"Skipping PX4 safe shutdown before GUI start: {exc}")
PY

graceful_stop_existing_hil

PX4_HIL_USE_SERIAL_HUB=0 \
PX4_HIL_USE_RT=0 \
PX4_HIL_PIN_CPUS=1 \
PX4_HIL_SIM_CPUSET="${PX4_HIL_SIM_CPUSET:-7}" \
"$SCRIPT_DIR/start_jmavsim_hitl.sh" "$PX4_ROOT" "$DEVICE" "$BAUDRATE"
