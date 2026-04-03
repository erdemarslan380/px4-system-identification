#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
DEVICE="${2:-/dev/ttyACM0}"
BAUDRATE="${3:-921600}"
ACTUATOR_RATE_HZ="${PX4_HIL_ACTUATOR_RATE_HZ:-50}"
SIM_PTY_LINK="${PX4_HIL_SIM_PTY:-/tmp/px4_hitl_sim.pty}"
CTRL_PTY_LINK="${PX4_HIL_CTRL_PTY:-/tmp/px4_hitl_ctrl.pty}"
USE_SERIAL_HUB="${PX4_HIL_USE_SERIAL_HUB:-1}"
MAVLINK_LOG_PATH="${PX4_HIL_MAVLINK_LOG:-}"
PIN_CPUS="${PX4_HIL_PIN_CPUS:-1}"
USE_RT="${PX4_HIL_USE_RT:-1}"
RT_PRIO_HUB="${PX4_HIL_RT_PRIO_HUB:-15}"
RT_PRIO_SIM="${PX4_HIL_RT_PRIO_SIM:-20}"

CPU_COUNT="$(nproc 2>/dev/null || echo 1)"
if [[ "$CPU_COUNT" -ge 4 ]]; then
  DEFAULT_HUB_CPU="$((CPU_COUNT - 2))"
  DEFAULT_SIM_CPU="$((CPU_COUNT - 1))"
else
  DEFAULT_HUB_CPU=""
  DEFAULT_SIM_CPU=""
fi

HUB_CPUSET="${PX4_HIL_HUB_CPUSET:-$DEFAULT_HUB_CPU}"
SIM_CPUSET="${PX4_HIL_SIM_CPUSET:-$DEFAULT_SIM_CPU}"
RT_AVAILABLE=0

if [[ "$USE_RT" == "1" ]] && command -v chrt >/dev/null 2>&1; then
  if chrt -r 1 /bin/true >/dev/null 2>&1; then
    RT_AVAILABLE=1
  else
    echo "Realtime scheduling not permitted for this user; falling back to taskset+ionice." >&2
  fi
fi

run_with_optional_sched() {
  local cpuset="$1"
  local rtprio="$2"
  shift
  shift

  local -a cmd=("$@")

  if [[ "$PIN_CPUS" == "1" ]] && [[ -n "$cpuset" ]] && command -v taskset >/dev/null 2>&1; then
    cmd=(taskset -c "$cpuset" "${cmd[@]}")
  fi

  if command -v ionice >/dev/null 2>&1; then
    cmd=(ionice -c2 -n0 "${cmd[@]}")
  fi

  if [[ "$RT_AVAILABLE" == "1" ]] && [[ -n "$rtprio" ]]; then
    cmd=(chrt -r "$rtprio" "${cmd[@]}")
  fi

  "${cmd[@]}"
}

if [[ "${PX4_SYSID_HEADLESS:-0}" == "1" ]]; then
  export HEADLESS=1
fi

if [[ ! -e "$DEVICE" ]]; then
  if [[ $# -lt 2 ]]; then
    shopt -s nullglob
    by_id_candidates=(/dev/serial/by-id/*CubeOrange*)
    shopt -u nullglob
    if [[ ${#by_id_candidates[@]} -eq 1 ]]; then
      echo "Default $DEVICE is not present; using ${by_id_candidates[0]} instead." >&2
      DEVICE="${by_id_candidates[0]}"
    fi
  fi
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

RESOLVED_DEVICE="$(readlink -f "$DEVICE" 2>/dev/null || echo "$DEVICE")"
if [[ "$RESOLVED_DEVICE" == /dev/ttyACM* ]]; then
  # Prevent Linux hangup-on-close behavior from bouncing the CubeOrange USB CDC
  # port to a new ttyACM index while jMAVSim is trying to open it.
  stty -F "$RESOLVED_DEVICE" -hupcl || true
  DEVICE="$RESOLVED_DEVICE"
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

if [[ "$USE_SERIAL_HUB" == "1" ]]; then
  HUB_CMD=(
    python3
    "$HOME/px4-system-identification/examples/mavlink_serial_hub.py"
    --serial-port "$DEVICE"
    --baud "$BAUDRATE"
    --sim-link "$SIM_PTY_LINK"
    --control-link "$CTRL_PTY_LINK"
  )

  if [[ -n "$MAVLINK_LOG_PATH" ]]; then
    HUB_CMD+=(--mavlink-log "$MAVLINK_LOG_PATH")
  fi

  run_with_optional_sched "$HUB_CPUSET" "$RT_PRIO_HUB" "${HUB_CMD[@]}" &
  HUB_PID=$!

  cleanup() {
    if [[ -n "${HUB_PID:-}" ]] && kill -0 "$HUB_PID" 2>/dev/null; then
      kill "$HUB_PID" 2>/dev/null || true
      wait "$HUB_PID" 2>/dev/null || true
    fi
  }

  trap cleanup EXIT INT TERM

  for _ in $(seq 1 50); do
    if [[ -L "$SIM_PTY_LINK" ]]; then
      break
    fi
    sleep 0.1
  done

  if [[ ! -L "$SIM_PTY_LINK" ]]; then
    echo "Failed to create simulator PTY at $SIM_PTY_LINK" >&2
    exit 3
  fi

  run_with_optional_sched "$SIM_CPUSET" "$RT_PRIO_SIM" \
    ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$SIM_PTY_LINK" -b "$BAUDRATE" -r "$ACTUATOR_RATE_HZ"
else
  run_with_optional_sched "$SIM_CPUSET" "$RT_PRIO_SIM" \
    ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d "$DEVICE" -b "$BAUDRATE" -r "$ACTUATOR_RATE_HZ"
fi
