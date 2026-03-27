Visual Gazebo SITL walkthrough
=============================

This is the shortest manual path for a first individual test.

1. Sync the overlay
-------------------
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
```

2. Start Gazebo SITL
--------------------
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If the build seems to reuse an older run:
```bash
shutdown
pkill -f '/PX4-Autopilot-Identification/build/px4_sitl_default/bin/px4' || true
pkill -f '/PX4-Autopilot-Identification/.*/gz/worlds/' || true
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If the GUI window does not appear:
```bash
gz sim -g
```

3. Start the helper modules in `pxh>`
-------------------------------------
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
trajectory_reader set_ident_profile hover_thrust
```

4. Use QGroundControl
---------------------
1. arm,
2. take off manually to about `3 m`,
3. stabilize hover,
4. switch to `OFFBOARD`.

5. Watch the PX4 console
------------------------
For every profile, the PX4 console prints:
- `Identification maneuver started: ...`
- `Purpose: ...`
- `Estimated duration: ...`
- `Identification maneuver completed: ...`
- `Identification log completed: ...`
- `Tracking log completed: ...`

Only send the next profile after the completion messages appear.

6. Run the remaining profiles
-----------------------------
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

7. Durations
------------
- `hover_thrust`: `26 s`
- `mass_vertical`: `36 s`
- `roll_sweep`: `28 s`
- `pitch_sweep`: `28 s`
- `yaw_sweep`: `24 s`
- `drag_x`: `30 s`
- `drag_y`: `30 s`
- `drag_z`: `30 s`
- `motor_step`: `24 s`

8. Refresh the shipped figures after the test
---------------------------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

The updated outputs appear here:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`
