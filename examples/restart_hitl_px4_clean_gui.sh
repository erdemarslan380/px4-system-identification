#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
BAUDRATE="${2:-921600}"
ENDPOINT="${3:-udpin:127.0.0.1:14550}"

graceful_stop_pattern() {
  local label="$1"
  local pattern="$2"
  local -a pids=()
  local parent_pid="${PPID:-0}"
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ "$pid" == "$$" ]] && continue
    [[ "$parent_pid" != "0" && "$pid" == "$parent_pid" ]] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" || true)
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

  echo "$label did not exit cleanly; refusing to restart HIL." >&2
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

find_cube_device() {
  local device=""
  shopt -s nullglob
  local cube_candidates=(/dev/serial/by-id/*CubeOrange* /dev/serial/by-id/*CubePilot_CubeOrange*)
  local tty_candidates=(/dev/ttyACM*)
  shopt -u nullglob

  for candidate in "${cube_candidates[@]}"; do
    case "$candidate" in
      *CubeOrange-BL*|*bootloader*)
        ;;
      *)
        device="$candidate"
        break
        ;;
    esac
  done

  if [[ -z "$device" && ${#tty_candidates[@]} -ge 1 ]]; then
    device="${tty_candidates[0]}"
  fi

  [[ -n "$device" ]] && printf '%s\n' "$device"
}

echo "Restarting HIL from a clean PX4 reboot"
echo "PX4 root : $PX4_ROOT"
echo "Endpoint : $ENDPOINT"
echo "Baud     : $BAUDRATE"

python3 - "$ENDPOINT" <<'PY' || true
import sys
import time
from pymavlink import mavutil

endpoint = sys.argv[1]

try:
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=3)
    if not hb:
        print("No live UDP heartbeat; skipping PX4 reboot")
        raise SystemExit(0)

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
    print("Sent PX4 safe shutdown over UDP")
except Exception as exc:
    print(f"Skipping PX4 reboot: {exc}")
PY

graceful_stop_existing_hil
sleep 2

SERIAL_DEVICE="$(find_cube_device || true)"
if [[ -n "$SERIAL_DEVICE" ]]; then
  python3 - "$SERIAL_DEVICE" "$BAUDRATE" <<'PY' || true
import sys
import time
from pymavlink import mavutil

device = sys.argv[1]
baud = int(sys.argv[2])

try:
    mav = mavutil.mavlink_connection(device, baud=baud, autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=3)
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
        print(f"Sent serial PX4 safe shutdown via {device}")
    else:
        print(f"No serial heartbeat on {device}; skipping serial reboot")
except Exception as exc:
    print(f"Skipping serial PX4 reboot on {device}: {exc}")
PY
  sleep 2
fi

DEVICE=""
for _ in $(seq 1 60); do
  shopt -s nullglob
  cube_candidates=(/dev/serial/by-id/*CubeOrange* /dev/serial/by-id/*CubePilot_CubeOrange*)
  tty_candidates=(/dev/ttyACM*)
  shopt -u nullglob

  for candidate in "${cube_candidates[@]}"; do
    case "$candidate" in
      *CubeOrange-BL*|*bootloader*)
        ;;
      *)
        DEVICE="$candidate"
        break 2
        ;;
    esac
  done

  if [[ -z "$DEVICE" && ${#tty_candidates[@]} -ge 1 ]]; then
    # Only fall back to ttyACM once the bootloader-only by-id node is gone.
    bootloader_seen=0
    for candidate in "${cube_candidates[@]}"; do
      case "$candidate" in
        *CubeOrange-BL*|*bootloader*)
          bootloader_seen=1
          ;;
      esac
    done
    if [[ $bootloader_seen -eq 0 ]]; then
      DEVICE="${tty_candidates[0]}"
      break
    fi
  fi
  sleep 1
done

if [[ -z "$DEVICE" ]]; then
  echo "CubeOrange did not re-enumerate after reboot" >&2
  exit 1
fi

LOG_DIR="${PX4_HIL_GUI_LOG_DIR:-/tmp}"
mkdir -p "$LOG_DIR"
GUI_LOG="$LOG_DIR/jmavsim_gui_$(date +%Y%m%d_%H%M%S).log"

if command -v setsid >/dev/null 2>&1; then
  nohup setsid "$SCRIPT_DIR/start_hitl_px4_baseline_gui.sh" "$PX4_ROOT" "$DEVICE" "$BAUDRATE" \
    >"$GUI_LOG" 2>&1 </dev/null &
else
  nohup "$SCRIPT_DIR/start_hitl_px4_baseline_gui.sh" "$PX4_ROOT" "$DEVICE" "$BAUDRATE" \
    >"$GUI_LOG" 2>&1 </dev/null &
fi
GUI_PID=$!
disown "$GUI_PID" 2>/dev/null || true

echo "Spawned visible jMAVSim launcher in background (pid=$GUI_PID)"
echo "GUI log  : $GUI_LOG"

for _ in $(seq 1 60); do
  if python3 - "$ENDPOINT" <<'PY' >/dev/null 2>&1
import sys
from pymavlink import mavutil

endpoint = sys.argv[1]
mav = mavutil.mavlink_connection(endpoint, autoreconnect=False)
hb = mav.wait_heartbeat(timeout=1.0)
raise SystemExit(0 if hb else 1)
PY
  then
    echo "UDP heartbeat detected; clean HIL session is ready"
    exit 0
  fi

  if ! kill -0 "$GUI_PID" 2>/dev/null; then
    echo "Visible jMAVSim launcher exited before heartbeat. See $GUI_LOG" >&2
    exit 1
  fi

  sleep 1
done

echo "Timed out waiting for UDP heartbeat from clean HIL session. See $GUI_LOG" >&2
exit 1
