#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
  echo "usage: $0 /path/to/PX4-Autopilot-Identification [relative/path/to/board.px4board]" >&2
  exit 1
fi

px4_root="$1"
board_rel="${2:-boards/px4/sitl/default.px4board}"
repo_root="$(cd "$(dirname "$0")" && pwd)"
px4_root_basename="$(basename "$px4_root")"

if [ ! -d "$px4_root" ]; then
  echo "PX4 root not found: $px4_root" >&2
  exit 1
fi

if [ "$px4_root_basename" = "PX4-Autopilot" ] && [ "${PX4_SYSID_ALLOW_SHARED_TREE:-0}" != "1" ]; then
  echo "Refusing to modify a shared PX4 tree: $px4_root" >&2
  echo "Use a dedicated clone such as ~/PX4-Autopilot-Identification." >&2
  echo "If you really intend to patch the shared tree, rerun with PX4_SYSID_ALLOW_SHARED_TREE=1." >&2
  exit 2
fi

rsync -a "$repo_root/overlay/" "$px4_root/"
python3 "$repo_root/experimental_validation/jmavsim_hil_patch.py" --px4-root "$px4_root" >/dev/null

cleanup_legacy_param_sources() {
  local module_dir="$1"
  [ -d "$module_dir" ] || return 0
  rm -f "$module_dir/custom_pos_control_params.c" "$module_dir/trajectory_reader_params.c"
}

patch_board_file() {
  local board_path="$1"
  [ -f "$board_path" ] || return 0
  python3 - "$board_path" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
required = [
    'CONFIG_MODULES_CUSTOM_POS_CONTROL=y',
    'CONFIG_MODULES_TRAJECTORY_READER=y',
    'CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y',
]
disabled = [
    'CONFIG_DRIVERS_ADC_ADS1115',
    'CONFIG_DRIVERS_BATT_SMBUS',
    'CONFIG_DRIVERS_CAMERA_CAPTURE',
    'CONFIG_DRIVERS_CAMERA_TRIGGER',
    'CONFIG_COMMON_DIFFERENTIAL_PRESSURE',
    'CONFIG_DRIVERS_GNSS_SEPTENTRIO',
    'CONFIG_DRIVERS_IMU_ANALOG_DEVICES_ADIS16448',
    'CONFIG_DRIVERS_IRLOCK',
    'CONFIG_DRIVERS_PCA9685_PWM_OUT',
    'CONFIG_DRIVERS_POWER_MONITOR_INA226',
    'CONFIG_DRIVERS_PWM_INPUT',
    'CONFIG_DRIVERS_SMART_BATTERY_BATMON',
    'CONFIG_DRIVERS_TRANSPONDER_SAGETECH_MXS',
    'CONFIG_DRIVERS_UAVCAN',
    'CONFIG_MODULES_AIRSPEED_SELECTOR',
    'CONFIG_MODULES_CAMERA_FEEDBACK',
    'CONFIG_MODULES_ESC_BATTERY',
    'CONFIG_MODULES_FW_ATT_CONTROL',
    'CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL',
    'CONFIG_MODULES_FW_MODE_MANAGER',
    'CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL',
    'CONFIG_MODULES_FW_RATE_CONTROL',
    'CONFIG_MODULES_GIMBAL',
    'CONFIG_MODULES_LANDING_TARGET_ESTIMATOR',
    'CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL',
    'CONFIG_MODULES_UXRCE_DDS_CLIENT',
    'CONFIG_MODULES_VTOL_ATT_CONTROL',
    'CONFIG_SYSTEMCMDS_BL_UPDATE',
    'CONFIG_SYSTEMCMDS_BSONDUMP',
    'CONFIG_SYSTEMCMDS_HARDFAULT_LOG',
    'CONFIG_SYSTEMCMDS_I2CDETECT',
    'CONFIG_SYSTEMCMDS_MFT',
]
changed = False
for line in required:
    if line not in text:
        text = text.rstrip() + '\n' + line + '\n'
        changed = True

for symbol in disabled:
    enabled_line = f'{symbol}=y'
    disabled_line = f'# {symbol} is not set'
    if enabled_line in text:
        text = text.replace(enabled_line, disabled_line)
        changed = True
    elif disabled_line not in text:
        text = text.rstrip() + '\n' + disabled_line + '\n'
        changed = True

if changed:
    path.write_text(text, encoding='utf-8')
PY
}

patch_gz_plugins_cmake() {
  local cmake_path="$1"
  [ -f "$cmake_path" ] || return 0
  python3 - "$cmake_path" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
changed = False
subdir_line = '    add_subdirectory(system_identification_logger)\n'
if 'add_subdirectory(system_identification_logger)' not in text:
    anchor = '    add_subdirectory(generic_motor)\n'
    if anchor in text:
        text = text.replace(anchor, anchor + subdir_line)
        changed = True
if 'SystemIdentificationLoggerPlugin' not in text and 'GenericMotorModelPlugin' in text:
    text = text.replace('GenericMotorModelPlugin BuoyancySystemPlugin', 'GenericMotorModelPlugin SystemIdentificationLoggerPlugin BuoyancySystemPlugin')
    changed = True
if changed:
    path.write_text(text, encoding='utf-8')
PY
}

patch_simulation_gazebo_script() {
  local script_path="$1"
  [ -f "$script_path" ] || return 0
  python3 - "$script_path" <<'PY'
from pathlib import Path
import sys
import re

path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
if "from pathlib import Path\n" not in text:
    text = text.replace("import shutil\n", "import shutil\nfrom pathlib import Path\n")

pattern = re.compile(
    r"(?ms)^        cmd = f'GZ_SIM_RESOURCE_PATH=\{args\.model_store\}/models '\n"
    r"(?:        .*?\n)*?"
    r"        cmd \+= f'GZ_SIM_SERVER_CONFIG_PATH=\{args\.model_store\}/server\.config '\n"
)
replacement = (
    "        px4_root = Path(__file__).resolve().parents[3]\n"
    "        plugin_root = px4_root / 'build' / 'px4_sitl_default' / 'src' / 'modules' / 'simulation' / 'gz_plugins'\n"
    "        truth_root = px4_root / 'build' / 'px4_sitl_default' / 'rootfs' / 'sysid_truth_logs'\n"
    "        cmd = f'GZ_SIM_RESOURCE_PATH={args.model_store}/models '\n"
    "        cmd += f'GZ_SIM_SYSTEM_PLUGIN_PATH={plugin_root} '\n"
    "        cmd += f'PX4_SYSID_LOG_DIR={truth_root} '\n"
    "        cmd += f'PX4_SYSID_LOG_SLOT=manual '\n"
    "        cmd += f'GZ_SIM_SERVER_CONFIG_PATH={args.model_store}/server.config '\n"
)
text = pattern.sub(replacement, text, count=1)
path.write_text(text, encoding='utf-8')
PY
}

patch_x500_model_sdf() {
  local model_path="$1"
  [ -f "$model_path" ] || return 0
  python3 - "$model_path" <<'PY'
from pathlib import Path
import sys
from xml.etree import ElementTree as ET


def indent(elem, level=0):
    text = "\n" + ("  " * level)
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = text + "  "
        for child in elem:
            indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = text
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = text


path = Path(sys.argv[1])
tree = ET.parse(path)
root = tree.getroot()
model = root.find(".//model")
if model is None:
    raise SystemExit(0)

for plugin in list(model.findall("plugin")):
    if plugin.attrib.get("filename") == "SystemIdentificationLoggerPlugin" or plugin.attrib.get("name") == "SystemIdentificationLoggerPlugin":
        model.remove(plugin)

rotor_plugins = [
    plugin for plugin in model.findall("plugin")
    if plugin.attrib.get("name") == "gz::sim::systems::MulticopterMotorModel"
]
if not rotor_plugins:
    raise SystemExit(0)

logger = ET.SubElement(model, "plugin", {
    "filename": "SystemIdentificationLoggerPlugin",
    "name": "SystemIdentificationLoggerPlugin",
})
ET.SubElement(logger, "enabled").text = "true"
ET.SubElement(logger, "base_link_name").text = "base_link"
ET.SubElement(logger, "sample_period_s").text = "0.005"
ET.SubElement(logger, "command_sub_topic").text = "command/motor_speed"

def copy_field(src_plugin, src_name, dst_parent, dst_name):
    node = src_plugin.find(src_name)
    if node is not None and node.text is not None:
        ET.SubElement(dst_parent, dst_name).text = node.text.strip()

for plugin in rotor_plugins:
    rotor = ET.SubElement(logger, "rotor")
    copy_field(plugin, "motorNumber", rotor, "motor_number")
    copy_field(plugin, "jointName", rotor, "joint_name")
    copy_field(plugin, "linkName", rotor, "link_name")
    copy_field(plugin, "turningDirection", rotor, "turning_direction")
    copy_field(plugin, "timeConstantUp", rotor, "time_constant_up_s")
    copy_field(plugin, "timeConstantDown", rotor, "time_constant_down_s")
    copy_field(plugin, "maxRotVelocity", rotor, "max_rot_velocity_radps")
    copy_field(plugin, "motorConstant", rotor, "motor_constant")
    copy_field(plugin, "momentConstant", rotor, "moment_constant")
    copy_field(plugin, "rotorDragCoefficient", rotor, "rotor_drag_coefficient")
    copy_field(plugin, "rollingMomentCoefficient", rotor, "rolling_moment_coefficient")
    copy_field(plugin, "rotorVelocitySlowdownSim", rotor, "rotor_velocity_slowdown_sim")

indent(root)
tree.write(path, encoding="utf-8", xml_declaration=True)
PY
}

patch_msg_cmake() {
  local cmake_path="$1"
  [ -f "$cmake_path" ] || return 0
  python3 - "$cmake_path" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
line = '\tMultiTrajectorySetpoint.msg\n'
if 'MultiTrajectorySetpoint.msg' not in text:
    for anchor in (
        '\tMissionResult.msg\n',
        '\tMission.msg\n',
        '\tManualControlSwitches.msg\n',
    ):
        if anchor in text:
            text = text.replace(anchor, anchor + line, 1)
            break
    else:
        marker = 'set(msg_files\n'
        if marker in text:
            text = text.replace(marker, marker + line, 1)
path.write_text(text, encoding='utf-8')
PY
}

patch_local_position_estimator_params() {
  local yaml_path="$1"
  [ -f "$yaml_path" ] || return 0
  python3 - "$yaml_path" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')

if '    LTEST_MODE:\n' in text:
    raise SystemExit(0)

anchor = (
    "    LPE_LT_COV:\n"
    "      description:\n"
    "        short: Minimum landing target standard covariance\n"
    "        long: Minimum landing target standard covariance, uses reported covariance\n"
    "          if greater\n"
    "      type: float\n"
    "      default: 0.0001\n"
    "      unit: m^2\n"
    "      min: 0.0\n"
    "      max: 10\n"
    "      decimal: 2\n"
)

insert = (
    anchor +
    "    LTEST_MODE:\n"
    "      description:\n"
    "        short: Landing target test mode\n"
    "        long: Select whether the landing target logic treats the target as moving or stationary.\n"
    "      type: int32\n"
    "      default: 0\n"
    "      min: 0\n"
    "      max: 1\n"
)

if anchor in text:
    text = text.replace(anchor, insert, 1)
else:
    fallback = "    LPE_FUSION:\n"
    if fallback not in text:
        raise SystemExit("could not patch local_position_estimator params.yaml")
    text = text.replace(
        fallback,
        "    LTEST_MODE:\n"
        "      description:\n"
        "        short: Landing target test mode\n"
        "        long: Select whether the landing target logic treats the target as moving or stationary.\n"
        "      type: int32\n"
        "      default: 0\n"
        "      min: 0\n"
        "      max: 1\n"
        + fallback,
        1,
    )

path.write_text(text, encoding='utf-8')
PY
}

patch_gz_bridge_gimbal() {
  local header_path="$1"
  local source_path="$2"
  [ -f "$header_path" ] || return 0
  [ -f "$source_path" ] || return 0
  python3 - "$header_path" "$source_path" <<'PY'
from pathlib import Path
import sys

header_path = Path(sys.argv[1])
source_path = Path(sys.argv[2])

header = header_path.read_text(encoding='utf-8')
if 'bool _gimbal_enabled{false};' not in header:
    header = header.replace(
        'GZGimbal _gimbal{_node};\n',
        'GZGimbal _gimbal{_node};\n\tbool _gimbal_enabled{false};\n',
        1,
    )
    header_path.write_text(header, encoding='utf-8')

source = source_path.read_text(encoding='utf-8')
old_init = (
    '\t// Gimbal mixing interface\n'
    '\tif (!_gimbal.init(_world_name, _model_name)) {\n'
    '\t\tPX4_ERR("failed to init gimbal");\n'
    '\t\treturn PX4_ERROR;\n'
    '\t}\n'
)
new_init = (
    '\t// Gimbal mixing interface\n'
    '\t_gimbal_enabled = _gimbal.init(_world_name, _model_name);\n'
    '\tif (!_gimbal_enabled) {\n'
    '\t\tPX4_WARN("failed to init gimbal, continuing without simulated gimbal");\n'
    '\t}\n'
)
if old_init in source:
    source = source.replace(old_init, new_init, 1)

source = source.replace('\t\t_gimbal.stop();\n', '\t\tif (_gimbal_enabled) {\n\t\t\t_gimbal.stop();\n\t\t}\n', 1)
source = source.replace('\t\t_gimbal.updateParams();\n', '\t\tif (_gimbal_enabled) {\n\t\t\t_gimbal.updateParams();\n\t\t}\n', 1)
source_path.write_text(source, encoding='utf-8')
PY
}

patch_rcs_hil_usb_stream_rate() {
  local rcS_path="$1"
  [ -f "$rcS_path" ] || return 0
  python3 - "$rcS_path" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
old = 'mavlink stream -d /dev/ttyACM0 -s HIL_ACTUATOR_CONTROLS -r 200'
new = 'mavlink stream -d /dev/ttyACM0 -s HIL_ACTUATOR_CONTROLS -r 50'
if old in text:
    text = text.replace(old, new, 1)
path.write_text(text, encoding='utf-8')
PY
}

patch_hil_airframe_startup() {
  local airframe_path="$1"
  [ -f "$airframe_path" ] || return 0
  python3 - "$airframe_path" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
text = text.replace('trajectory_reader abs_ref 0 0 -3 0', 'trajectory_reader ref 0 0 -3 0')
path.write_text(text, encoding='utf-8')
PY
}

patch_board_file "$px4_root/$board_rel"
patch_gz_plugins_cmake "$px4_root/src/modules/simulation/gz_plugins/CMakeLists.txt"
patch_simulation_gazebo_script "$px4_root/Tools/simulation/gz/simulation-gazebo"
patch_x500_model_sdf "$px4_root/Tools/simulation/gz/models/x500/model.sdf"
patch_msg_cmake "$px4_root/msg/CMakeLists.txt"
patch_gz_bridge_gimbal \
  "$px4_root/src/modules/simulation/gz_bridge/GZBridge.hpp" \
  "$px4_root/src/modules/simulation/gz_bridge/GZBridge.cpp"
patch_local_position_estimator_params "$px4_root/src/modules/local_position_estimator/params.yaml"
patch_rcs_hil_usb_stream_rate "$px4_root/ROMFS/px4fmu_common/init.d/rcS"
patch_hil_airframe_startup "$px4_root/ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil"
cleanup_legacy_param_sources "$px4_root/src/modules/custom_pos_control"
cleanup_legacy_param_sources "$px4_root/src/modules/trajectory_reader"

echo "Overlay copied into $px4_root"
echo "Board flags ensured in $board_rel"
echo "Gazebo plugin CMake patched for SystemIdentificationLoggerPlugin"
echo "Gazebo launcher patched for GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "x500 model.sdf patched for SystemIdentificationLoggerPlugin"
echo "uORB message list patched for MultiTrajectorySetpoint.msg"
echo "gz_bridge patched to tolerate missing simulated gimbal topics"
echo "local_position_estimator params.yaml patched for LTEST_MODE compatibility"
echo "rcS patched to lower USB HIL_ACTUATOR_CONTROLS stream rate for CDC-based HITL"
echo "1001_rc_quad_x.hil patched to use relative hover setup for HITL"
echo "Legacy params.c overlays removed in favor of module.yaml"
echo "Next step: build SITL with 'make px4_sitl gz_x500' or your target board build."
