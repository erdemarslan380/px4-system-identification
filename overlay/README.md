Overlay contents
================

This folder is copied into an upstream PX4 workspace.

Included modules
- `src/modules/custom_pos_control/`
  - minimal forwarding module for `px4_default` and `sysid`
- `src/modules/trajectory_reader/`
  - baseline reference generator and built-in identification motions
- `src/modules/simulation/gz_plugins/system_identification_logger/`
  - Gazebo truth logger plugin

Important note
- The overlay is intentionally small.
- The full PX4 source tree still comes from upstream PX4.
- Use `../sync_into_px4_workspace.sh` to copy the overlay and patch the minimal build hooks.
