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

Then run one full HIL campaign:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign full_stack \
  --prepare-hover \
  --allow-missing-local-position \
  --blind-hover-seconds 12 \
  --timeout 520
```

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
