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
]
changed = False
for line in required:
    if line not in text:
        text = text.rstrip() + '\n' + line + '\n'
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

patch_board_file "$px4_root/$board_rel"
patch_gz_plugins_cmake "$px4_root/src/modules/simulation/gz_plugins/CMakeLists.txt"
patch_simulation_gazebo_script "$px4_root/Tools/simulation/gz/simulation-gazebo"
patch_x500_model_sdf "$px4_root/Tools/simulation/gz/models/x500/model.sdf"
patch_msg_cmake "$px4_root/msg/CMakeLists.txt"
cleanup_legacy_param_sources "$px4_root/src/modules/custom_pos_control"
cleanup_legacy_param_sources "$px4_root/src/modules/trajectory_reader"

echo "Overlay copied into $px4_root"
echo "Board flags ensured in $board_rel"
echo "Gazebo plugin CMake patched for SystemIdentificationLoggerPlugin"
echo "Gazebo launcher patched for GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "x500 model.sdf patched for SystemIdentificationLoggerPlugin"
echo "uORB message list patched for MultiTrajectorySetpoint.msg"
echo "Legacy params.c overlays removed in favor of module.yaml"
echo "Next step: build SITL with 'make px4_sitl gz_x500' or your target board build."
