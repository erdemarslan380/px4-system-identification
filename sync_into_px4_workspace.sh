#!/usr/bin/env bash
set -euo pipefail
if [ $# -ne 1 ]; then
  echo "usage: $0 /path/to/PX4-Autopilot" >&2
  exit 1
fi
px4_root="$1"
repo_root="$(cd "$(dirname "$0")" && pwd)"
rsync -a "$repo_root/overlay/" "$px4_root/"
echo "Overlay copied into $px4_root"
