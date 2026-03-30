Operator Quickstart
===================

This is the shortest copy-paste path.

Use this file if:
- you already know PX4 basics,
- you want the minimum number of commands,
- you do not want to read the long technical document first.

Follow the stages in order:
1. `SITL`
2. `HIL`
3. `real flight`

If a stage is not clean, stop there and fix it before continuing.

1. Prepare the dedicated PX4 tree
---------------------------------
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
python3 experimental_validation/validation_trajectories.py \
  --trajectories-dir ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories
```

2. Prepare the CubeOrange SD card
---------------------------------
With the SD card mounted on the workstation:

```bash
cd ~/px4-system-identification
./examples/prepare_sdcard_payload.sh /media/$USER/<sdcard_mount_name>
```

This installs `id_100..104.traj` and creates `tracking_logs/` and `identification_logs/` on the card.
On hardware the trajectory binaries must end up under `/fs/microsd/trajectories/`.

3. Start SITL
-------------
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If you see a missing `LTEST_MODE` parameter error in `local_position_estimator`, run:
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
rm -rf ~/PX4-Autopilot-Identification/build/px4_sitl_default

cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

4. One full campaign in SITL
----------------------------
QGroundControl is optional in SITL. If you open it, do it only after Gazebo is already running and keep it as a passive viewer.

SITL order:
1. start Gazebo SITL,
2. optionally open QGroundControl only to watch,
3. run the campaign helper from a second terminal,
4. close QGroundControl fully before switching to HIL.

In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
param set COM_DISARM_PRFLT 60
trajectory_reader ref 0 0 -3 0
```

From a second terminal:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign full_stack \
  --prepare-hover \
  --heartbeat-warmup 5 \
  --arm-attempts 10 \
  --timeout 520
```

This one command runs:
- 9 identification maneuvers
- then trajectories `100..104`

Return-to-anchor legs are not logged.

Only the 9 identification maneuvers:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign identification_only \
  --prepare-hover \
  --heartbeat-warmup 5 \
  --arm-attempts 10 \
  --timeout 420
```

Only the 5 trajectories:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign trajectory_only \
  --prepare-hover \
  --heartbeat-warmup 5 \
  --arm-attempts 10 \
  --timeout 220
```

One identification maneuver:
```bash
cd ~/px4-system-identification
python3 examples/run_hitl_udp_sequence.py \
  --endpoint udpin:127.0.0.1:14550 \
  --kind ident \
  --name hover_thrust
```

One trajectory:
```bash
cd ~/px4-system-identification
python3 examples/run_hitl_udp_sequence.py \
  --endpoint udpin:127.0.0.1:14550 \
  --kind trajectory \
  --traj-id 100
```

Build the candidate after the identification part:
```bash
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

RC workflow option
------------------
If you want to run HIL or real flight from the transmitter instead of a helper script, configure:
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

Press the `H` button to apply the currently selected workflow.

QGroundControl mapping check:
- in `Vehicle Setup > Radio`, complete the normal radio calibration and note which spare controls appear as `AUX 1..6`,
- set `CST_RC_CTRL_CH`, `TRJ_RC_MODE_CH`, and `TRJ_RC_SEL_CH` from those `AUX` slot numbers,
- if the `H` trigger comes through QGroundControl manual-control or joystick input, set it in `Vehicle Setup > Joystick` and use that one-based button index for `TRJ_RC_START_BTN`,
- verify the final mapping with `listener manual_control_setpoint`: the pots should move the expected `auxN` field and the `H` trigger must toggle `buttons`.

Keep these separate:
- `Vehicle Setup > Flight Modes`: normal PX4 mode switch such as `Position` and `Offboard`
- repo RC params above: select one ident, one trajectory, or one campaign

Receiver in HIL:
- yes, the physical receiver can stay connected while `jMAVSim` runs,
- `jMAVSim` uses the USB CDC simulator link, not the RC input pins,
- this is the right way to test electrical mode-switch behavior before real flight.

If the `H` trigger does not affect `manual_control_setpoint.buttons`, keep using the helper script or shell command to start the selected maneuver or campaign.

5. Refresh the shipped figures
------------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

Outputs:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`

6. HIL/HITL smoke order
-----------------------
Use HIL only to prove that one uninterrupted campaign works in one boot and writes CSV logs to the SD card.

If QGroundControl was open for SITL, close it fully before HIL.

HIL order:
1. connect the board over USB,
2. open QGroundControl once to set `SYS_AUTOSTART = 1001` and `SYS_HITL = 1`,
3. reboot the board and close QGroundControl fully,
4. start `jMAVSim` on the current USB CDC device,
5. only then reopen QGroundControl in UDP-only mode if you want a viewer,
6. run the HIL helper from a second terminal.

Upload:
```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

Find the current serial devices:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Use QGroundControl over USB once to set:
- `SYS_AUTOSTART = 1001`
- `SYS_HITL = 1`

Then reboot the board and close QGroundControl fully.

Start jMAVSim on the current USB CDC device:
```bash
cd ~/px4-system-identification
./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification <usb_cdc_device> 921600
```

After jMAVSim is already running, QGroundControl may be opened again only in UDP-only mode.

If you want to test the physical receiver in HIL:
1. connect the receiver before boot,
2. in `Vehicle Setup > Radio`, complete radio calibration,
3. in `Vehicle Setup > Flight Modes`, assign one switch position to `Position` and one to `Offboard`,
4. set `COM_RC_IN_MODE = 0`,
5. reboot, close QGC, start `jMAVSim`, then reopen QGC only in UDP-only mode.

Minimal shell state for RC `Position <-> Offboard` smoke tests:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader ref 0 0 -3 0
```

Then:
1. arm in `Position`,
2. climb to hover,
3. flip the transmitter flight-mode switch to `Offboard`,
4. flip back to `Position`,
5. verify the nav-state change in QGC or with `listener vehicle_status`.

Then run one full HIL campaign:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign full_stack \
  --prepare-hover \
  --manual-control-mode 4 \
  --allow-missing-local-position \
  --blind-hover-seconds 12 \
  --timeout 520
```

Use `--manual-control-mode 0` instead if you want the physical receiver to remain active during the scripted HIL campaign. Keep the sticks centered; leaving `Offboard` from the transmitter will abort the running campaign.

Only the 9 HIL identification maneuvers:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign identification_only \
  --prepare-hover \
  --allow-missing-local-position \
  --blind-hover-seconds 12 \
  --timeout 420
```

Only the 5 HIL trajectories:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign trajectory_only \
  --prepare-hover \
  --allow-missing-local-position \
  --blind-hover-seconds 12 \
  --timeout 220
```

One HIL identification maneuver:
```bash
cd ~/px4-system-identification
python3 examples/run_hitl_udp_sequence.py \
  --endpoint udpin:127.0.0.1:14550 \
  --kind ident \
  --name hover_thrust \
  --allow-missing-local-position \
  --blind-hover-seconds 12
```

One HIL trajectory:
```bash
cd ~/px4-system-identification
python3 examples/run_hitl_udp_sequence.py \
  --endpoint udpin:127.0.0.1:14550 \
  --kind trajectory \
  --traj-id 100 \
  --allow-missing-local-position \
  --blind-hover-seconds 12
```

Acceptance check on the SD card:
- `9` files in `identification_logs/`
- `14` files in `tracking_logs/`

HIL RAM/CPU report after the run:
```bash
cd ~/px4-system-identification
python3 examples/pull_sdcard_logs_over_mavftp.py \
  --port <usb_cdc_device> \
  --baud 57600 \
  --destination-dir ~/px4-system-identification/hitl_runs/session_001

python3 experimental_validation/report_hil_resources.py \
  --ulg ~/px4-system-identification/hitl_runs/session_001/ulg/<latest>.ulg \
  --out ~/px4-system-identification/hitl_runs/session_001/hil_resource_summary.json
```

Current checked repo example:
- [live_check_001_summary.json](/home/earsub/px4-system-identification/examples/hil_resource_report/live_check_001_summary.json)
- current verdict: not signed off yet, because one checked HIL run logged `RAM usage too high`, `low on stack`, and `parameters verify: failed`.

7. Pull SD-card logs into the repo and review them
--------------------------------------------------
```bash
cd ~/px4-system-identification
./examples/import_sdcard_logs.sh /media/$USER/<sdcard_mount_name> \
  ~/px4-system-identification/hitl_runs/session_001

python3 examples/pull_sdcard_logs_over_mavftp.py \
  --port <usb_cdc_device> \
  --baud 57600 \
  --destination-dir ~/px4-system-identification/hitl_runs/session_001

python3 experimental_validation/build_hitl_review_bundle.py \
  --log-root ~/px4-system-identification/hitl_runs/session_001 \
  --out-dir ~/px4-system-identification/hitl_runs/session_001/review
```

Use the mounted-SD script or the USB CDC MAVFTP pull script for a given session, not both.
Before the live pull, close `jMAVSim`, `QGroundControl`, and any USB MAVLink shell on the same USB CDC device.
The MAVFTP pull helper writes `pull_report.json`, skips already complete files, and only retries the missing ones on the next run.

8. Keep calibration across firmware updates
-------------------------------------------
Primary path: use QGroundControl
1. connect the board,
2. open `Parameters`,
3. open `Tools`,
4. click `Save to file`,
5. save over:
   - `experimental_validation/qgc/current_vehicle.params`
6. regenerate the restore files:
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore \
  --board-defaults overlay/ROMFS/px4fmu_common/init.d/rc.board_defaults
```

Fallback one-command path if MAVLink is already available:
```bash
cd ~/px4-system-identification
./examples/update_vehicle_calibration_snapshot.sh udpin:127.0.0.1:14550 57600
```

This updates:
- `experimental_validation/qgc/current_vehicle.params`
- `experimental_validation/qgc/restore/restore_calibration.params`
- `experimental_validation/qgc/restore/restore_calibration.nsh`
- `overlay/ROMFS/px4fmu_common/init.d/rc.board_defaults`

After that, rebuild firmware normally:
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/cubepilot/cubeorange/default.px4board

cd ~/PX4-Autopilot-Identification
make cubepilot_cubeorange_default
```

If you recalibrate later in QGroundControl:
1. use `Parameters > Tools > Save to file`,
2. replace `experimental_validation/qgc/current_vehicle.params`,
3. rerun:
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore \
  --board-defaults overlay/ROMFS/px4fmu_common/init.d/rc.board_defaults
```

Do not bake the entire parameter dump into firmware defaults. Keep the full dump for reference, but only restore the calibration- and RC-related subset through `rc.board_defaults`.

If you are not removing the SD card, first find the live USB CDC device:
```bash
ls /dev/ttyACM*
```

If you want to browse before pulling:
```bash
cd ~/px4-system-identification
python3 examples/sdcard_browser.py \
  --serial-port <usb_cdc_device> \
  --baud 57600
```

Open:
- `http://127.0.0.1:8765/`

The browser can list `/fs/microsd`, open folders, and pull selected files into:
- `~/px4-system-identification/hitl_runs/browser_downloads/`

Open:
- `~/px4-system-identification/hitl_runs/session_001/review/index.html`

HIL trajectory control:
- `TRJ_ACTIVE_ID = 100..104`
- `TRJ_MODE_CMD = 1` starts the selected trajectory
- `TRJ_MODE_CMD = 0` returns to position hold
- `TRJ_MODE_CMD = 2` starts the selected identification profile
- `TRJ_IDENT_PROF = 0..8` selects the identification profile
- `CST_POS_CTRL_TYP = 4` keeps the PX4 baseline, `6` switches to the identification path
