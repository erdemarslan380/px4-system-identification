Operator Quickstart
===================

This is the shortest path for a fresh terminal.

1. Prepare the dedicated PX4 tree
---------------------------------
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

3. Identification in SITL
-------------------------
In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
```

Then in QGroundControl:
1. arm
2. take off to about `3 m`
3. hover
4. switch to `OFFBOARD`

Run:
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

Build the candidate:
```bash
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

4. Validation trajectories in SITL
----------------------------------
In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

Trajectory IDs:
- `100`: `hairpin`, `23 s`
- `101`: `lemniscate`, `19 s`
- `102`: `circle`, `15 s`
- `103`: `time_optimal_30s`, `11 s`
- `104`: `minimum_snap_50s`, `14 s`

Run one trajectory:
```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

After each run:
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

Outputs:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`

6. HIL/HITL quick order
-----------------------
- upload the CubeOrange firmware first
- keep `jMAVSim`, `QGroundControl`, and `mavlink_shell.py` closed during upload
- start `jMAVSim` on USB `ttyACM0`
- open QGroundControl in UDP-only mode
- if a shell is needed, use FTDI `ttyUSB0`

jMAVSim:
```bash
cd ~/px4-system-identification
./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification /dev/ttyACM0 921600
```

Optional shell:
```bash
python3 ~/PX4-Autopilot-Identification/Tools/mavlink_shell.py /dev/ttyUSB0 -b 57600
```

HIL trajectory control:
- `TRJ_ACTIVE_ID = 100..104`
- `TRJ_MODE_CMD = 1` starts the selected trajectory
- `TRJ_MODE_CMD = 0` returns to position hold
