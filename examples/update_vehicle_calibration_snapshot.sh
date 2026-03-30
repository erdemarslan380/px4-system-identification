#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENDPOINT="${1:-udpin:127.0.0.1:14550}"
BAUD="${2:-57600}"
QGC_DUMP="${REPO_ROOT}/experimental_validation/qgc/current_vehicle.params"
RESTORE_DIR="${REPO_ROOT}/experimental_validation/qgc/restore"
BOARD_DEFAULTS="${REPO_ROOT}/overlay/ROMFS/px4fmu_common/init.d/rc.board_defaults"

cd "${REPO_ROOT}"

python3 experimental_validation/export_vehicle_params.py \
  --endpoint "${ENDPOINT}" \
  --baud "${BAUD}" \
  --out "${QGC_DUMP}"

python3 experimental_validation/calibration_restore.py \
  --input "${QGC_DUMP}" \
  --out-dir "${RESTORE_DIR}" \
  --board-defaults "${BOARD_DEFAULTS}"

echo "Updated:"
echo "  ${QGC_DUMP}"
echo "  ${RESTORE_DIR}"
echo "  ${BOARD_DEFAULTS}"
