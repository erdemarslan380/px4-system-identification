#!/usr/bin/env bash
set -euo pipefail

TARGET_ROOT="${1:-$HOME/PX4-Autopilot-Identification}"
REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"

if [ ! -d "$TARGET_ROOT/.git" ]; then
  echo "Cloning PX4 into $TARGET_ROOT"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$TARGET_ROOT"
else
  echo "Using existing PX4 workspace: $TARGET_ROOT"
fi

echo "Syncing overlay into $TARGET_ROOT"
"$REPO_ROOT/sync_into_px4_workspace.sh" "$TARGET_ROOT"

cat <<EOF

Dedicated identification workspace is ready:
  $TARGET_ROOT

If this is a fresh machine, run once:
  cd "$TARGET_ROOT"
  bash ./Tools/setup/ubuntu.sh

Then build SITL with:
  cd "$TARGET_ROOT"
  make px4_sitl gz_x500
EOF
