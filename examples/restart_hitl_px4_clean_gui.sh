#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
BAUDRATE="${2:-921600}"
ENDPOINT="${3:-udpin:127.0.0.1:14550}"
MAVFTP_BAUDRATE="${PX4_HIL_MAVFTP_BAUDRATE:-57600}"
SDCARD_INSTALLER="${SCRIPT_DIR}/install_hitl_sdcard_payload_over_mavftp.py"
USB_STREAM_SCRIPT="${SCRIPT_DIR}/set_hitl_usb_actuator_stream.sh"
INSTALL_SDCARD_PAYLOAD="${PX4_HIL_INSTALL_SDCARD_PAYLOAD:-0}"
APPLY_HIL_PARAM_BASELINE="${PX4_HIL_APPLY_HIL_PARAM_BASELINE:-0}"
APPLY_JMAVSIM_IRIS_GEOMETRY="${PX4_HIL_APPLY_JMAVSIM_IRIS_GEOMETRY:-0}"
ENFORCE_HIL_CORE_PARAMS="${PX4_HIL_ENFORCE_CORE_PARAMS:-1}"
HIL_IMU_INTEG_RATE_HZ="${PX4_HIL_IMU_INTEG_RATE_HZ:-250}"
HIL_RATE_LOOP_HZ="${PX4_HIL_RATE_LOOP_HZ:-1000}"
HIL_ATTITUDE_LOOP_HZ="${PX4_HIL_ATTITUDE_LOOP_HZ:-250}"
HIL_POSITION_LOOP_HZ="${PX4_HIL_POSITION_LOOP_HZ:-100}"
HIL_USB_STREAM_RATE_HZ="${PX4_HIL_USB_STREAM_RATE_HZ:-${PX4_HIL_ACTUATOR_RATE_HZ:-200}}"
STARTUP_TIMEOUT_S="${PX4_HIL_STARTUP_TIMEOUT_S:-90}"
READY_SOAK_S="${PX4_HIL_READY_SOAK_S:-3}"

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

  echo "$label did not exit cleanly; sending SIGKILL." >&2
  kill -KILL "${pids[@]}" 2>/dev/null || true

  for _ in $(seq 1 20); do
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
    sleep 0.1
  done

  echo "$label could not be stopped cleanly." >&2
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
    echo "jMAVSim GUI window was not found; falling back to process stop." >&2
    kill -TERM "${pids[@]}" 2>/dev/null || true
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

  echo "jMAVSim did not exit from GUI close; sending SIGKILL." >&2
  kill -KILL "${pids[@]}" 2>/dev/null || true

  for _ in $(seq 1 20); do
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
    sleep 0.1
  done

  echo "jMAVSim process could not be stopped cleanly." >&2
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
for _ in $(seq 1 "$STARTUP_TIMEOUT_S"); do
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

if [[ "$ENFORCE_HIL_CORE_PARAMS" == "1" ]]; then
echo "Enforcing core HITL startup parameters"
PX4_HIL_IMU_INTEG_RATE_HZ="$HIL_IMU_INTEG_RATE_HZ" \
PX4_HIL_RATE_LOOP_HZ="$HIL_RATE_LOOP_HZ" \
PX4_HIL_ATTITUDE_LOOP_HZ="$HIL_ATTITUDE_LOOP_HZ" \
PX4_HIL_POSITION_LOOP_HZ="$HIL_POSITION_LOOP_HZ" \
python3 - "$DEVICE" "$BAUDRATE" <<'PY'
import sys
import time
import math
import struct
import os
from pymavlink import mavutil

device = sys.argv[1]
baud = int(sys.argv[2])
imu_integ_rate_hz = int(os.environ["PX4_HIL_IMU_INTEG_RATE_HZ"])
rate_loop_hz = int(os.environ["PX4_HIL_RATE_LOOP_HZ"])

params = [
    ("SYS_AUTOSTART", 1001, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("SYS_HITL", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    # Keep the HIL airframe on direct simulator state injection. We align the
    # timing/filter chain with the real-flight baseline, but estimator parity
    # is a separate change because 1001_rc_quad_x.hil explicitly bypasses EKF2.
    ("EKF2_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("SYS_HAS_BARO", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_INTEG_RATE", imu_integ_rate_hz, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_GYRO_RATEMAX", rate_loop_hz, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_GYRO_CUTOFF", 40.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("IMU_DGYRO_CUTOFF", 20.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("MAV_0_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("MAV_0_FORWARD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("MAV_0_RATE", 1200, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("COM_ARM_WO_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC1", 101, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC2", 102, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC3", 103, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC4", 104, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
]

# Neutralize real-hardware sensor calibration snapshots in HIL. The CubeOrange
# board defaults file can contain physical-vehicle offsets that bias HIL sensor
# data and lead to asymmetric motor commands even at rest.
for idx in range(4):
    params.extend(
        [
            (f"CAL_GYRO{idx}_XOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_GYRO{idx}_YOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_GYRO{idx}_ZOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_XOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_YOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_ZOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_XSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_YSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_ACC{idx}_ZSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_BARO{idx}_OFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_XOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_YOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_ZOFF", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_XSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_YSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_ZSCALE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_XODIAG", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_YODIAG", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_ZODIAG", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_XCOMP", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_YCOMP", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (f"CAL_MAG{idx}_ZCOMP", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ]
    )


def norm_param_id(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode(errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def decode_param_value(raw_value: float, param_type: int):
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        packed = struct.pack("<f", float(raw_value))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<b", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<B", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<h", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<H", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<i", packed)[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<I", packed)[0]
    return float(raw_value)


def encode_param_value(value, param_type: int) -> float:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        int_value = int(round(float(value)))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<f", struct.pack("<bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<f", struct.pack("<Bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<f", struct.pack("<hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<f", struct.pack("<Hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<f", struct.pack("<i", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<f", struct.pack("<I", int_value))[0]
    return float(value)


def param_value_matches(raw_value: float, expected, param_type: int) -> bool:
    if math.isclose(float(raw_value), float(expected), rel_tol=0.0, abs_tol=1e-3):
        return True
    decoded = decode_param_value(raw_value, param_type)
    if isinstance(decoded, int):
        return decoded == int(round(float(expected)))
    return math.isclose(float(decoded), float(expected), rel_tol=0.0, abs_tol=1e-3)


def set_param(master, name: str, value, param_type: int, timeout: float = 4.0) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        encode_param_value(value, param_type),
        param_type,
    )
    deadline = time.time() + timeout
    last_value = None
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        if norm_param_id(msg.param_id) != name:
            continue
        last_value = msg.param_value
        if param_value_matches(msg.param_value, value, param_type):
            return
    raise TimeoutError(f"PARAM_VALUE confirmation missing/mismatched for {name} (last={last_value!r}, expected={value!r})")


try:
    mav = mavutil.mavlink_connection(device, baud=baud, autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=6)
    if not hb:
        raise RuntimeError("heartbeat missing on CubeOrange after reboot")

    for name, value, param_type in params:
        set_param(mav, name, value, param_type)

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    time.sleep(1.5)

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    print("Core HITL params saved; rebooting once more")
except Exception as exc:
    print(f"Warning: could not enforce core HITL params: {exc}")
PY
sleep 6

DEVICE=""
for _ in $(seq 1 60); do
  DEVICE="$(find_cube_device || true)"
  if [[ -n "$DEVICE" ]]; then
    break
  fi
  sleep 1
done

if [[ -z "$DEVICE" ]]; then
  echo "CubeOrange did not re-enumerate after core-param reboot" >&2
  exit 1
fi
fi

if [[ "$INSTALL_SDCARD_PAYLOAD" == "1" && -f "$SDCARD_INSTALLER" ]]; then
  echo "Installing HITL SD card trajectory payload via MAVFTP"
  python3 "$SDCARD_INSTALLER" --port "$DEVICE" --baud "$MAVFTP_BAUDRATE"
fi

if [[ "$APPLY_HIL_PARAM_BASELINE" == "1" ]]; then
echo "Applying HIL parameter baseline"
PX4_HIL_IMU_INTEG_RATE_HZ="$HIL_IMU_INTEG_RATE_HZ" \
PX4_HIL_RATE_LOOP_HZ="$HIL_RATE_LOOP_HZ" \
PX4_HIL_ATTITUDE_LOOP_HZ="$HIL_ATTITUDE_LOOP_HZ" \
PX4_HIL_POSITION_LOOP_HZ="$HIL_POSITION_LOOP_HZ" \
python3 - "$DEVICE" "$BAUDRATE" <<'PY'
import sys
import time
import math
import struct
import os
from pymavlink import mavutil

device = sys.argv[1]
baud = int(sys.argv[2])
apply_iris_geometry = False
apply_iris_geometry = os.environ.get("PX4_HIL_APPLY_JMAVSIM_IRIS_GEOMETRY", "0") == "1"
imu_integ_rate_hz = int(os.environ["PX4_HIL_IMU_INTEG_RATE_HZ"])
rate_loop_hz = int(os.environ["PX4_HIL_RATE_LOOP_HZ"])

params = [
    ("SYS_AUTOSTART", 1001, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("SYS_HITL", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("EKF2_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("SYS_HAS_BARO", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_INTEG_RATE", imu_integ_rate_hz, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_GYRO_RATEMAX", rate_loop_hz, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("IMU_GYRO_CUTOFF", 40.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("IMU_DGYRO_CUTOFF", 20.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("MAV_0_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("MAV_0_FORWARD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("MAV_0_RATE", 1200, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC1", 101, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC2", 102, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC3", 103, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("HIL_ACT_FUNC4", 104, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
]

if apply_iris_geometry:
    params.extend(
        [
            ("CA_AIRFRAME", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CA_ROTOR_COUNT", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("CA_ROTOR0_PX", 0.1515, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR0_PY", 0.2450, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR0_KM", 0.05, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR1_PX", -0.1515, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR1_PY", -0.1875, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR1_KM", 0.05, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR2_PX", 0.1515, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR2_PY", -0.2450, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR2_KM", -0.05, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR3_PX", -0.1515, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR3_PY", 0.1875, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("CA_ROTOR3_KM", -0.05, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ]
    )


def norm_param_id(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode(errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def decode_param_value(raw_value: float, param_type: int):
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        packed = struct.pack("<f", float(raw_value))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<b", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<B", packed[:1])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<h", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<H", packed[:2])[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<i", packed)[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<I", packed)[0]
    return float(raw_value)


def encode_param_value(value, param_type: int) -> float:
    if param_type in (
        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    ):
        int_value = int(round(float(value)))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<f", struct.pack("<bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<f", struct.pack("<Bxxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<f", struct.pack("<hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<f", struct.pack("<Hxx", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<f", struct.pack("<i", int_value))[0]
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<f", struct.pack("<I", int_value))[0]
    return float(value)


def param_value_matches(raw_value: float, expected, param_type: int) -> bool:
    if math.isclose(float(raw_value), float(expected), rel_tol=0.0, abs_tol=1e-3):
        return True
    decoded = decode_param_value(raw_value, param_type)
    if isinstance(decoded, int):
        return decoded == int(round(float(expected)))
    return math.isclose(float(decoded), float(expected), rel_tol=0.0, abs_tol=1e-3)


def set_param(master, name: str, value, param_type: int, timeout: float = 4.0) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        encode_param_value(value, param_type),
        param_type,
    )
    deadline = time.time() + timeout
    last_value = None
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue
        if norm_param_id(msg.param_id) != name:
            continue
        last_value = msg.param_value
        if param_value_matches(msg.param_value, value, param_type):
            return
    raise TimeoutError(f"PARAM_VALUE confirmation missing/mismatched for {name} (last={last_value!r}, expected={value!r})")


try:
    mav = mavutil.mavlink_connection(device, baud=baud, autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=6)
    if not hb:
        raise RuntimeError("heartbeat missing on CubeOrange after reboot")

    for name, value, param_type in params:
        set_param(mav, name, value, param_type)

    label = "jMAVSim Iris" if apply_iris_geometry else "generic HIL quad-X"
    print(f"Applied {label} HIL parameter baseline")
except Exception as exc:
    print(f"Warning: could not apply HIL parameter baseline: {exc}")
PY
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
    if [[ -x "$USB_STREAM_SCRIPT" ]]; then
      PX4_HIL_ENDPOINT="$ENDPOINT" "$USB_STREAM_SCRIPT" "$HIL_USB_STREAM_RATE_HZ" "$ENDPOINT" >/dev/null 2>&1 || \
        echo "Warning: could not set HIL_ACTUATOR_CONTROLS stream to ${HIL_USB_STREAM_RATE_HZ} Hz" >&2
    fi

    sleep "$READY_SOAK_S"

    echo "UDP heartbeat detected; clean HIL session is ready"
    exit 0
  fi

  if ! kill -0 "$GUI_PID" 2>/dev/null; then
    if ! pgrep -f 'java.*jmavsim_run.jar|jmavsim_run.sh -q -s -d |mavlink_serial_hub.py|start_hitl_px4_baseline_gui.sh' >/dev/null 2>&1; then
      echo "Visible jMAVSim launcher exited before heartbeat and no jMAVSim process is alive. See $GUI_LOG" >&2
      exit 1
    fi

    echo "Visible launcher exited, but jMAVSim is still alive; continuing to wait for heartbeat"
  fi

  if ! pgrep -f 'java.*jmavsim_run.jar|jmavsim_run.sh -q -s -d |mavlink_serial_hub.py|start_hitl_px4_baseline_gui.sh' >/dev/null 2>&1; then
    echo "jMAVSim exited before heartbeat. See $GUI_LOG" >&2
    exit 1
  fi

  sleep 1
done

echo "Timed out waiting for UDP heartbeat from clean HIL session. See $GUI_LOG" >&2
exit 1
