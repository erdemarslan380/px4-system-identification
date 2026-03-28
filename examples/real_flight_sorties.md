Real-flight trajectory and identification plan
=============================================

CubeOrange firmware build
-------------------------
Build the hardware firmware in the dedicated workspace before the flight day:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/cubepilot/cubeorange/default.px4board

cd ~/PX4-Autopilot-Identification
make cubepilot_cubeorange_default
```

With FTDI disconnected, you can upload from the terminal:
- `make cubepilot_cubeorange_default upload`

If FTDI stays connected, use the targeted uploader command:

```bash
cd ~/PX4-Autopilot-Identification
python3 Tools/px4_uploader.py \
  --port /dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00 \
  build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4
```

You can also flash this file from QGroundControl with the CubeOrange connected over USB:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

Keep this same firmware on the board for the first USB-connected HIL/HITL check.
The HIL/HITL wiring and simulator steps are described in the main README.

Use one calm day and keep the same simple rule in every sortie:
- manual takeoff,
- stabilize at about `3 m`,
- switch to `OFFBOARD`,
- run one or two commands,
- return to hover,
- land.

1. Identification sorties
-------------------------
Start the helper modules on the vehicle:

```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
```

Then fly these four sorties.

Sortie 1
- `trajectory_reader set_ident_profile hover_thrust`
- `trajectory_reader set_ident_profile mass_vertical`

Sortie 2
- `trajectory_reader set_ident_profile roll_sweep`
- `trajectory_reader set_ident_profile pitch_sweep`

Sortie 3
- `trajectory_reader set_ident_profile yaw_sweep`
- `trajectory_reader set_ident_profile motor_step`

Sortie 4
- `trajectory_reader set_ident_profile drag_x`
- `trajectory_reader set_ident_profile drag_y`
- `trajectory_reader set_ident_profile drag_z`

Approximate durations:
- `hover_thrust`: `26 s`
- `mass_vertical`: `36 s`
- `roll_sweep`: `28 s`
- `pitch_sweep`: `28 s`
- `yaw_sweep`: `24 s`
- `drag_x`: `30 s`
- `drag_y`: `30 s`
- `drag_z`: `30 s`
- `motor_step`: `24 s`

Wait for these PX4 messages before the next command:
- `Identification maneuver completed: ...`
- `Identification log completed: ...`
- `Tracking log completed: ...`

Logs are written to:
- `/fs/microsd/identification_logs/`
- `/fs/microsd/tracking_logs/`

2. Validation trajectories on the real vehicle
----------------------------------------------
Switch to the baseline PX4 controller path:

```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

The five validation trajectories are:
- `100`: `hairpin`, `23 s`
- `101`: `lemniscate`, `19 s`
- `102`: `circle`, `15 s`
- `103`: `time_optimal_30s`, `11 s`
- `104`: `minimum_snap_50s`, `14 s`

They share the same anchored start hover at `(0, 0, -3)`, but they do not all return to that point on their own. After each run, command position hold again before starting the next ID.

Run them with this pattern:

```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

After one trajectory finishes:

```bash
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

Then run the next ID.

The five full command blocks are:

```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory

trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 101
trajectory_reader set_mode trajectory

trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 102
trajectory_reader set_mode trajectory

trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 103
trajectory_reader set_mode trajectory

trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 104
trajectory_reader set_mode trajectory
```

Validation tracking logs are written to:
- `/fs/microsd/tracking_logs/`

3. After the flights
--------------------
1. copy the identification logs and tracking logs to your workstation,
2. estimate the identified model,
3. write the candidate into the Gazebo SDF,
4. run the same five validation trajectories in SITL,
5. overlay the real-flight traces and the digital-twin traces,
6. regenerate the same figures.
