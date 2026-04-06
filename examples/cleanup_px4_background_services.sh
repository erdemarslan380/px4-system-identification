#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
RESET_USB=0
DEVICE_ARG=""
declare -A EXCLUDED_PIDS=()

usage() {
  cat <<EOF
Usage:
  $SCRIPT_NAME [--device /dev/ttyACM0] [--reset-usb]

What it does:
  - closes PX4 SITL, Gazebo, jMAVSim
  - stops common PX4 helper scripts from this repo
  - releases busy serial devices such as /dev/ttyACM* and /dev/ttyUSB*
  - optionally tries a best-effort USB re-enumeration for one device

Examples:
  $SCRIPT_NAME
  $SCRIPT_NAME --device /dev/ttyACM0
  $SCRIPT_NAME --device /dev/ttyACM0 --reset-usb
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)
      DEVICE_ARG="${2:-}"
      shift 2
      ;;
    --reset-usb)
      RESET_USB=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

graceful_stop_pattern() {
  local label="$1"
  local pattern="$2"
  local -a pids=()

  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ -n "${EXCLUDED_PIDS[$pid]:-}" ]] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" || true)

  if [[ ${#pids[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Stopping $label: ${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null || true

  for _ in $(seq 1 40); do
    local alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    [[ $alive -eq 0 ]] && return 0
    sleep 0.25
  done

  echo "Force killing $label: ${pids[*]}"
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

collect_ancestor_pids() {
  local pid="$$"
  local parent=""

  while [[ -n "$pid" && "$pid" != "0" ]]; do
    EXCLUDED_PIDS["$pid"]=1
    parent="$(ps -o ppid= -p "$pid" 2>/dev/null | awk '{print $1}' || true)"
    [[ -z "$parent" || "$parent" == "$pid" ]] && break
    pid="$parent"
  done
}

close_windows_by_regex() {
  local regex="$1"
  local -a win_ids=()
  if command -v wmctrl >/dev/null 2>&1; then
    mapfile -t win_ids < <(wmctrl -lx 2>/dev/null | awk "tolower(\$0) ~ /$regex/ {print \$1}")
    if ((${#win_ids[@]} > 0)); then
      echo "Closing windows via wmctrl: ${win_ids[*]}"
      for wid in "${win_ids[@]}"; do
        wmctrl -ic "$wid" || true
      done
      sleep 1
    fi
  fi

  if command -v xdotool >/dev/null 2>&1; then
    mapfile -t win_ids < <(xdotool search --all --name "$regex" 2>/dev/null || true)
    if ((${#win_ids[@]} > 0)); then
      echo "Closing windows via xdotool: ${win_ids[*]}"
      for wid in "${win_ids[@]}"; do
        xdotool windowclose "$wid" || true
      done
      sleep 1
    fi
  fi
}

collect_devices() {
  local -a devices=()
  local candidate=""

  if [[ -n "$DEVICE_ARG" ]]; then
    devices+=("$DEVICE_ARG")
  fi

  shopt -s nullglob
  for candidate in /dev/serial/by-id/*CubeOrange* /dev/serial/by-id/*CubePilot* /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$candidate" ]] || continue
    devices+=("$candidate")
  done
  shopt -u nullglob

  if [[ ${#devices[@]} -eq 0 ]]; then
    return 0
  fi

  printf '%s\n' "${devices[@]}" | awk '!seen[$0]++'
}

release_serial_users() {
  local device="$1"
  [[ -e "$device" ]] || return 0

  if command -v fuser >/dev/null 2>&1; then
    echo "Releasing serial users on $device"
    fuser -k "$device" 2>/dev/null || true
  fi
}

reset_usb_device() {
  local device="$1"
  local sys_path=""
  local walk=""

  [[ -e "$device" ]] || return 0

  if ! command -v udevadm >/dev/null 2>&1; then
    echo "Skipping USB reset for $device: udevadm not found"
    return 0
  fi

  sys_path="$(udevadm info -q path -n "$device" 2>/dev/null || true)"
  if [[ -z "$sys_path" ]]; then
    echo "Skipping USB reset for $device: could not resolve sysfs path"
    return 0
  fi

  walk="/sys$sys_path"
  while [[ "$walk" != "/" ]]; do
    if [[ -f "$walk/authorized" ]]; then
      if [[ ! -w "$walk/authorized" ]]; then
        echo "Skipping USB reset for $device: $walk/authorized is not writable"
        return 0
      fi
      echo "Best-effort USB reset for $device via $walk"
      echo 0 > "$walk/authorized"
      sleep 1
      echo 1 > "$walk/authorized"
      udevadm settle || true
      return 0
    fi
    walk="$(dirname "$walk")"
  done

  echo "Skipping USB reset for $device: no writable USB authorization node found"
}

echo "Cleaning PX4 background services"
collect_ancestor_pids

close_windows_by_regex "px4 sitl log|px4 sitl console|px4 console"
close_windows_by_regex "px4 gazebo nested display"
close_windows_by_regex "jmavsim"

graceful_stop_pattern "PX4 SITL" '/PX4-Autopilot-Identification/build/px4_sitl_default/bin/px4'
graceful_stop_pattern "Gazebo" '(^|[[:space:]])gz sim([[:space:]]|$)'
graceful_stop_pattern "Nested Gazebo display" 'Xephyr .*PX4 Gazebo Nested Display'
graceful_stop_pattern "PX4 console tails" 'tail .*px4_console\.log'
graceful_stop_pattern "jMAVSim" 'java.*jmavsim_run.jar|jmavsim_run.sh -q -s -d '
graceful_stop_pattern "MAVLink serial hub" 'mavlink_serial_hub.py'
graceful_stop_pattern "HITL/SITL helpers" 'run_hitl_|run_mavlink_campaign.py|run_sitl_validation.py'

while IFS= read -r device; do
  [[ -z "$device" ]] && continue
  release_serial_users "$device"
  if [[ $RESET_USB -eq 1 ]]; then
    reset_usb_device "$device"
  fi
done < <(collect_devices || true)

rm -f /tmp/px4_lock-0 /tmp/px4-sock-0

echo "Cleanup complete"
