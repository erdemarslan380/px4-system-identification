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
patch_msg_cmake "$px4_root/msg/CMakeLists.txt"
cleanup_legacy_param_sources "$px4_root/src/modules/custom_pos_control"
cleanup_legacy_param_sources "$px4_root/src/modules/trajectory_reader"

echo "Overlay copied into $px4_root"
echo "Board flags ensured in $board_rel"
echo "Gazebo plugin CMake patched for SystemIdentificationLoggerPlugin"
echo "uORB message list patched for MultiTrajectorySetpoint.msg"
echo "Legacy params.c overlays removed in favor of module.yaml"
echo "Next step: build SITL with 'make px4_sitl gz_x500' or your target board build."
