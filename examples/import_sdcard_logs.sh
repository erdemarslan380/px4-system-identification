#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <mounted_sdcard_root> [destination_dir]" >&2
  exit 1
fi

SD_ROOT="${1%/}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST_DIR="${2:-${REPO_ROOT}/hitl_runs/${STAMP}}"

if [[ ! -d "${SD_ROOT}" ]]; then
  echo "SD card mount point not found: ${SD_ROOT}" >&2
  exit 2
fi

mkdir -p "${DEST_DIR}/tracking_logs" "${DEST_DIR}/identification_logs"

if compgen -G "${SD_ROOT}/tracking_logs/*.csv" > /dev/null; then
  cp "${SD_ROOT}/tracking_logs/"*.csv "${DEST_DIR}/tracking_logs/"
fi

if compgen -G "${SD_ROOT}/identification_logs/*.csv" > /dev/null; then
  cp "${SD_ROOT}/identification_logs/"*.csv "${DEST_DIR}/identification_logs/"
fi

sync

echo "Imported logs into: ${DEST_DIR}"
echo "Tracking logs:"
find "${DEST_DIR}/tracking_logs" -maxdepth 1 -type f -name '*.csv' -printf '  %f\n' | sort || true
echo "Identification logs:"
find "${DEST_DIR}/identification_logs" -maxdepth 1 -type f -name '*.csv' -printf '  %f\n' | sort || true
