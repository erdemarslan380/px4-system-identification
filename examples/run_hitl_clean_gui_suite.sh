#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
OUT_ROOT="${2:-$PWD/hitl_runs/clean_gui_suite_$(date +%Y%m%d_%H%M%S)}"
MAX_ATTEMPTS="${PX4_HIL_SUITE_MAX_ATTEMPTS:-2}"

export PX4_HIL_TRAJ_HOVER_Z="${PX4_HIL_TRAJ_HOVER_Z:--5.0}"
export PX4_HIL_TRAJ_TILT_LIMIT_DEG="${PX4_HIL_TRAJ_TILT_LIMIT_DEG:-60}"
export PX4_HIL_TRAJ_Z_TOLERANCE="${PX4_HIL_TRAJ_Z_TOLERANCE:-3.0}"

mkdir -p "$OUT_ROOT"
STATUS_FILE="$OUT_ROOT/status.tsv"
printf 'traj_id\tattempt\tstatus\tlog_path\n' > "$STATUS_FILE"

run_one() {
  local traj_id="$1"
  local attempt=""
  local log_path=""

  for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    log_path="$OUT_ROOT/traj_${traj_id}_attempt${attempt}.log"
    echo "===== trajectory ${traj_id} attempt ${attempt}/${MAX_ATTEMPTS} ====="
    if "$SCRIPT_DIR/run_hitl_clean_gui_trajectory.sh" "$PX4_ROOT" "$traj_id" 2>&1 | tee "$log_path"; then
      printf '%s\t%s\tok\t%s\n' "$traj_id" "$attempt" "$log_path" >> "$STATUS_FILE"
      return 0
    fi
    printf '%s\t%s\tfailed\t%s\n' "$traj_id" "$attempt" "$log_path" >> "$STATUS_FILE"
    sleep 5
  done

  return 1
}

overall_status=0
for traj_id in 100 101 102 103 104; do
  if ! run_one "$traj_id"; then
    overall_status=1
  fi
done

echo "Suite logs   : $OUT_ROOT"
echo "Suite status : $STATUS_FILE"
exit "$overall_status"
