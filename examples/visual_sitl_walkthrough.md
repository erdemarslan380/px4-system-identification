Visual Gazebo SITL walkthrough
=============================

1. Install the shipped validation trajectories
----------------------------------------------
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
python3 experimental_validation/validation_trajectories.py \
  --trajectories-dir ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories
```

2. Start SITL
-------------
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

3. Run identification
---------------------
In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
```

Then in QGroundControl:
1. arm,
2. take off manually to about `3 m`,
3. stabilize hover,
4. switch to `OFFBOARD`.

Run the profiles one by one:
```bash
trajectory_reader set_ident_profile hover_thrust
trajectory_reader set_ident_profile mass_vertical
trajectory_reader set_ident_profile roll_sweep
trajectory_reader set_ident_profile pitch_sweep
trajectory_reader set_ident_profile yaw_sweep
trajectory_reader set_ident_profile drag_x
trajectory_reader set_ident_profile drag_y
trajectory_reader set_ident_profile drag_z
trajectory_reader set_ident_profile motor_step
```

4. Run the five trajectories in SITL
------------------------------------
Switch to the baseline PX4 controller path:
```bash
custom_pos_control set px4_default
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

Then run any of the five trajectories:
```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

Trajectory IDs:
- `100`: `hairpin`, `23 s`
- `101`: `lemniscate`, `19 s`
- `102`: `circle`, `15 s`
- `103`: `time_optimal_30s`, `11 s`
- `104`: `minimum_snap_50s`, `14 s`

All five start from the same anchored hover pose at `(0, 0, -3)`, but they do not all end there. Return to position hold before launching the next one.

After one trajectory finishes:
```bash
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

5. Refresh the shipped figures
------------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```
