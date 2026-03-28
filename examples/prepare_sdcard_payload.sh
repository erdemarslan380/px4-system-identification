#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <mounted_sdcard_root> [trajectory_source_dir]" >&2
  exit 1
fi

SD_ROOT="${1%/}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TRAJ_SRC="${2:-${REPO_ROOT}/assets/validation_trajectories}"

if [[ ! -d "${SD_ROOT}" ]]; then
  echo "SD card mount point not found: ${SD_ROOT}" >&2
  exit 2
fi

if [[ ! -d "${TRAJ_SRC}" ]]; then
  echo "Trajectory source directory not found: ${TRAJ_SRC}" >&2
  exit 3
fi

mkdir -p \
  "${SD_ROOT}/trajectories" \
  "${SD_ROOT}/tracking_logs" \
  "${SD_ROOT}/identification_logs"

for traj_id in 100 101 102 103 104; do
  src="${TRAJ_SRC}/id_${traj_id}.traj"
  dst="${SD_ROOT}/trajectories/id_${traj_id}.traj"
  if [[ ! -f "${src}" ]]; then
    echo "Missing trajectory asset: ${src}" >&2
    exit 4
  fi
  cp "${src}" "${dst}"
done

sync

echo "SD card payload prepared under: ${SD_ROOT}"
echo "Installed trajectories:"
printf '  %s\n' \
  "${SD_ROOT}/trajectories/id_100.traj" \
  "${SD_ROOT}/trajectories/id_101.traj" \
  "${SD_ROOT}/trajectories/id_102.traj" \
  "${SD_ROOT}/trajectories/id_103.traj" \
  "${SD_ROOT}/trajectories/id_104.traj"
echo "Log directories:"
printf '  %s\n' \
  "${SD_ROOT}/tracking_logs" \
  "${SD_ROOT}/identification_logs"
