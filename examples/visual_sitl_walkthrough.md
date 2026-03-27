Visual Gazebo SITL walkthrough
=============================

1. Prepare the dedicated PX4 workspace
```bash
cd ~/px4-system-identification
./prepare_identification_workspace.sh ~/PX4-Autopilot-Identification
```

2. Sync the overlay into the dedicated PX4 tree again if you changed this repository
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
```

3. Build and start SITL
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

Healthy signs in the terminal:
- `Linking CXX executable bin/px4`
- `INFO [init] Gazebo simulator ...`
- `INFO [init] Starting gazebo with world: ...`
- `pxh>`

4. If the Gazebo GUI window does not appear, open it manually
```bash
gz sim -g
```

5. Start the helper modules in `pxh>`
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
trajectory_reader set_ident_profile hover_thrust
```

6. Arm and take off from QGroundControl
- arm normally
- take off manually to about `3 m`
- stabilize hover
- switch to `OFFBOARD`

7. What you should see in the PX4 console when a profile runs
- `Identification profile set to ...`
- `Purpose: ...`
- `Estimated duration: ...`
- `Identification maneuver started: ...`
- `Identification log started: ...`
- `Identification maneuver completed: ...`
- `Identification log completed: ...`
- `Tracking log completed: ...`

That final pair means the current maneuver has finished and the vehicle is holding the final reference.

8. Run the remaining profiles one by one
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

9. Approximate durations
- `hover_thrust`: `26 s`
- `mass_vertical`: `36 s`
- `roll_sweep`: `28 s`
- `pitch_sweep`: `28 s`
- `yaw_sweep`: `24 s`
- `drag_x`: `30 s`
- `drag_y`: `30 s`
- `drag_z`: `30 s`
- `motor_step`: `24 s`

10. Exact SITL log folders
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/`

11. Estimate from the latest logs
```bash
LATEST_IDENT=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/*.csv | head -n 1)
LATEST_TRUTH=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/*.csv | head -n 1)

cd ~/px4-system-identification
python3 experimental_validation/cli.py \
  --csv "$LATEST_IDENT" \
  --truth-csv "$LATEST_TRUTH" \
  --ident-log \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/session_001
```
