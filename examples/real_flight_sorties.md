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
- start one built-in campaign or one single item,
- let the vehicle return to the common anchor between items,
- land after the last trajectory.

RC-driven operator workflow
---------------------------
If you want to run the sortie from the transmitter instead of typing commands in the shell, configure:
- `CST_RC_SEL_EN = 1`
- `CST_RC_CTRL_CH = <controller_pot>`
- `TRJ_RC_MODE_EN = 1`
- `TRJ_RC_MODE_CH = <workflow_pot>`
- `TRJ_RC_SEL_EN = 1`
- `TRJ_RC_SEL_CH = <item_pot>`
- `TRJ_RC_MIN_ID = 100`
- `TRJ_RC_MAX_ID = 104`
- `TRJ_RC_START_EN = 1`
- `TRJ_RC_START_BTN = <H_button_index>`

Workflow pot slots:
- `0`: hold position
- `1`: one identification maneuver
- `2`: one trajectory
- `3`: `identification_only`
- `4`: `trajectory_only`
- `5`: `full_stack`

With slot `1`, the item pot selects one of the `9` identification profiles.
With slot `2`, the item pot selects one trajectory in the shipped range `100..104`.
Press the `H` button to apply the current selection.

QGroundControl calibration check:
- in `Vehicle Setup > Radio`, finish the normal radio calibration and note which spare controls show up as `AUX 1..6`,
- use those `AUX` slot numbers for `CST_RC_CTRL_CH`, `TRJ_RC_MODE_CH`, and `TRJ_RC_SEL_CH`,
- if the `H` trigger comes through QGroundControl manual-control input, set it in `Vehicle Setup > Joystick` and use that one-based button index for `TRJ_RC_START_BTN`,
- on the vehicle, run `listener manual_control_setpoint` once and verify that the pots move the expected `auxN` fields and that the `H` trigger toggles `buttons`.

If the `H` trigger does not affect `manual_control_setpoint.buttons`, keep the RC pots for selection only and start the selected maneuver or campaign from the helper script or shell.

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

2. Identification-only real-flight campaign
-------------------------------------------
```bash
trajectory_reader set_campaign identification_only
trajectory_reader start_campaign
```

Expected logs:
- `9` files under `/fs/microsd/identification_logs/`
- `9` files under `/fs/microsd/tracking_logs/`

3. Trajectory-only real-flight campaign
---------------------------------------
```bash
trajectory_reader set_campaign trajectory_only
trajectory_reader start_campaign
```

Expected logs:
- `5` files under `/fs/microsd/tracking_logs/`

4. One identification maneuver
------------------------------
```bash
custom_pos_control set sysid
trajectory_reader set_ident_profile hover_thrust
trajectory_reader set_mode identification
```

5. One trajectory
-----------------
```bash
custom_pos_control set px4_default
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

6. After the flight
-------------------
1. copy the identification logs and tracking logs to your workstation,
2. import the SD-card CSV files into this repository:
   - `cd ~/px4-system-identification`
   - `./examples/import_sdcard_logs.sh /media/$USER/<sdcard_mount_name> ~/px4-system-identification/flight_runs/session_001`
   - or, for a live pull without removing the card over USB CDC:
     - `ls /dev/ttyACM*`
     - `python3 examples/pull_sdcard_logs_over_mavftp.py --port <usb_cdc_device> --baud 57600 --destination-dir ~/px4-system-identification/flight_runs/session_001`
   - or, to browse first and then pull selected files:
     - `python3 examples/sdcard_browser.py --serial-port <usb_cdc_device> --baud 57600`
     - open `http://127.0.0.1:8765/`
3. build the interactive review bundle:
   - `python3 experimental_validation/build_hitl_review_bundle.py --log-root ~/px4-system-identification/flight_runs/session_001 --out-dir ~/px4-system-identification/flight_runs/session_001/review`
4. if you also pulled the PX4 `.ulg` files, summarize board RAM/CPU with:
   - `python3 experimental_validation/report_hil_resources.py --ulg ~/px4-system-identification/flight_runs/session_001/ulg/<latest>.ulg --out ~/px4-system-identification/flight_runs/session_001/hil_resource_summary.json`
5. open:
   - `~/px4-system-identification/flight_runs/session_001/review/index.html`
6. estimate the identified model,
7. write the candidate into the Gazebo SDF,
8. run the same five validation trajectories in SITL,
9. overlay the real-flight traces and the digital-twin traces,
10. regenerate the same figures.

Before the live USB CDC pull or the browser flow:
- close `jMAVSim`,
- close `QGroundControl`,
- close any MAVLink shell on the same USB CDC device.

The repository already includes the current baseline real-flight PID validation traces under:
- `~/px4-system-identification/examples/real_flight_baseline_pid/tracking_logs/`
