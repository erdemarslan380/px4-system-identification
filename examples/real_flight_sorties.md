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

Upload from the terminal with:

```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

You can also flash this file from QGroundControl with the CubeOrange connected over USB:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

Keep this same firmware on the board for the first USB-connected HIL/HITL check.
The HIL/HITL wiring and simulator steps are described in the main README.

Before the flight day, prepare the SD card on the workstation:

```bash
cd ~/px4-system-identification
./examples/prepare_sdcard_payload.sh /media/$USER/<sdcard_mount_name>
```

This must leave these paths on the card:
- `/fs/microsd/trajectories/id_100.traj`
- `/fs/microsd/trajectories/id_101.traj`
- `/fs/microsd/trajectories/id_102.traj`
- `/fs/microsd/trajectories/id_103.traj`
- `/fs/microsd/trajectories/id_104.traj`
- `/fs/microsd/tracking_logs/`
- `/fs/microsd/identification_logs/`

Use one calm day and keep the workflow simple:
- manual takeoff,
- stabilize at about `3 m`,
- switch to `OFFBOARD`,
- start one built-in campaign,
- let the vehicle return to the common anchor between items,
- land after the last trajectory.

1. One full real-flight campaign
--------------------------------
Start the helper modules on the vehicle:

```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader ref 0 0 -3 0
trajectory_reader set_campaign full_stack
```

Then start the campaign:

```bash
trajectory_reader start_campaign
```

The built-in `full_stack` order is:
1. `hover_thrust`
2. `mass_vertical`
3. `roll_sweep`
4. `pitch_sweep`
5. `yaw_sweep`
6. `drag_x`
7. `drag_y`
8. `drag_z`
9. `motor_step`
10. trajectory `100` hairpin
11. trajectory `101` lemniscate
12. trajectory `102` circle
13. trajectory `103` time_optimal_30s
14. trajectory `104` minimum_snap_50s

The campaign returns to the common anchor between items and those return legs are not logged.

Expected logs after one uninterrupted sortie:
- `9` files under `/fs/microsd/identification_logs/`
- `14` files under `/fs/microsd/tracking_logs/`

2. After the flight
--------------------
1. copy the identification logs and tracking logs to your workstation,
2. import the SD-card CSV files into this repository:
   - `cd ~/px4-system-identification`
   - `./examples/import_sdcard_logs.sh /media/$USER/<sdcard_mount_name> ~/px4-system-identification/hitl_runs/session_001`
3. build the interactive review bundle:
   - `python3 experimental_validation/build_hitl_review_bundle.py --log-root ~/px4-system-identification/hitl_runs/session_001 --out-dir ~/px4-system-identification/hitl_runs/session_001/review`
4. open:
   - `~/px4-system-identification/hitl_runs/session_001/review/index.html`
5. estimate the identified model,
6. write the candidate into the Gazebo SDF,
7. run the same five validation trajectories in SITL,
8. overlay the real-flight traces and the digital-twin traces,
9. regenerate the same figures.
