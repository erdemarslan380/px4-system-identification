Visual Gazebo SITL checklist
===========================

1. Apply the overlay
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot
```

2. Build PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

3. Relaunch with GUI enabled
```bash
cd ~/PX4-Autopilot
unset HEADLESS
make px4_sitl gz_x500
```

4. If no GUI window appears, open it manually
```bash
gz sim -g
```

5. In the PX4 shell
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
trajectory_reader set_ident_profile hover_thrust
```

6. Change profiles when the vehicle is stable
```bash
trajectory_reader set_ident_profile mass_vertical
trajectory_reader set_ident_profile roll_sweep
trajectory_reader set_ident_profile pitch_sweep
trajectory_reader set_ident_profile yaw_sweep
trajectory_reader set_ident_profile drag_x
trajectory_reader set_ident_profile drag_y
trajectory_reader set_ident_profile drag_z
trajectory_reader set_ident_profile motor_step
```

7. Export logs after each session
- PX4 identification logs:
  - `build/px4_sitl_default/rootfs/identification_logs/`
- PX4 tracking logs:
  - `build/px4_sitl_default/rootfs/tracking_logs/`
- Gazebo truth logs:
  - `build/px4_sitl_default/rootfs/sysid_truth_logs/`
