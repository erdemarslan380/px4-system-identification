#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
USB_STREAM_HZ="${PX4_HIL_USB_STREAM_HZ:-200}"

RUN_ROOT="${1:-$REPO_ROOT/hitl_runs/all_five_traj_20260409_usb${USB_STREAM_HZ}}"
PX4_ROOT="${2:-/home/earsub/PX4-Autopilot-Identification}"
ENDPOINT="${3:-udpin:127.0.0.1:14550}"

RESTART_SCRIPT="$SCRIPT_DIR/restart_hitl_px4_clean_gui.sh"
USB_STREAM_SCRIPT="$SCRIPT_DIR/set_hitl_usb_actuator_stream.sh"
TRAJ_HELPER="$SCRIPT_DIR/run_hitl_px4_builtin_trajectory_minimal.py"

mkdir -p "$RUN_ROOT"
SUMMARY_FILE="$RUN_ROOT/summary.ndjson"
: >"$SUMMARY_FILE"

CONTROL_PORT="${PX4_HIL_CTRL_PTY:-/tmp/px4_hitl_ctrl.pty}"

chmod +x "$USB_STREAM_SCRIPT"

emit_summary() {
  local traj_id="$1"
  local name="$2"
  local restart_rc="$3"
  local stream_rc="$4"
  local run_rc="$5"
  local stable="$6"
  local failsafe="$7"
  local summary="$8"

  python3 - "$traj_id" "$name" "$restart_rc" "$stream_rc" "$run_rc" "$stable" "$failsafe" "$summary" <<'PY'
import json
import sys

traj_id, name, restart_rc, stream_rc, run_rc, stable, failsafe, summary = sys.argv[1:]
obj = {
    "traj_id": int(traj_id),
    "name": name,
    "restart_rc": int(restart_rc),
    "stream_rc": int(stream_rc),
    "run_rc": int(run_rc),
    "stable": bool(int(stable)),
    "failsafe": bool(int(failsafe)),
    "summary": summary,
}
print(json.dumps(obj, ensure_ascii=True))
PY
}

run_one() {
  local traj_id="$1"
  local name="$2"
  local case_dir="$RUN_ROOT/$name"
  local restart_log="$case_dir/restart.log"
  local stream_log="$case_dir/usb_stream.log"
  local run_log="$case_dir/run.log"
  local restart_rc=999
  local stream_rc=999
  local run_rc=999
  local stable=0
  local failsafe=0
  local summary=""

  mkdir -p "$case_dir"

  PX4_HIL_ACTUATOR_RATE_HZ=250 "$RESTART_SCRIPT" "$PX4_ROOT" 921600 "$ENDPOINT" >"$restart_log" 2>&1
  restart_rc=$?

  if [[ "$restart_rc" -eq 0 ]]; then
    "$USB_STREAM_SCRIPT" "$USB_STREAM_HZ" \
      >"$stream_log" 2>&1
    stream_rc=$?

    PX4_HIL_ACTUATOR_RATE_HZ=250 python3 "$TRAJ_HELPER" \
      --endpoint "$ENDPOINT" \
      --traj-id "$traj_id" \
      --ramp-seconds 8 \
      >"$run_log" 2>&1
    run_rc=$?

    if grep -q "trajectory_reader built-in trajectory stable" "$run_log"; then
      stable=1
    fi
    if grep -q "Failsafe activated" "$run_log"; then
      failsafe=1
    fi
    summary="$(grep -F "Built-in trajectory summary:" "$run_log" | tail -n 1 || true)"
  fi

  emit_summary "$traj_id" "$name" "$restart_rc" "$stream_rc" "$run_rc" "$stable" "$failsafe" "$summary" | tee -a "$SUMMARY_FILE"
}

run_one 100 hairpin
run_one 101 lemniscate
run_one 102 circle
run_one 103 time_optimal_30s
run_one 104 minimum_snap_50s

echo "SUMMARY_FILE=$SUMMARY_FILE"
