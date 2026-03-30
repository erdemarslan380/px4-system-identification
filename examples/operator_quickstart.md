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

Build the candidate after the identification part:
```bash
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
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

6. HIL/HITL smoke order
-----------------------
Use HIL only to prove that one uninterrupted campaign works in one boot and writes CSV logs to the SD card.

Upload:
```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

jMAVSim:
```bash
cd ~/px4-system-identification
./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification /dev/ttyACM0 921600
```

Then run the same campaign:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign full_stack \
  --timeout 520
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
  --port /dev/ttyACM0 \
  --baud 57600 \
  --destination-dir ~/px4-system-identification/hitl_runs/session_001

python3 experimental_validation/build_hitl_review_bundle.py \
  --log-root ~/px4-system-identification/hitl_runs/session_001 \
  --out-dir ~/px4-system-identification/hitl_runs/session_001/review
```

Use the mounted-SD script or the USB CDC MAVFTP pull script for a given session, not both.
Before the live pull, close `jMAVSim`, `QGroundControl`, and any USB MAVLink shell on `/dev/ttyACM0`.
The MAVFTP pull helper writes `pull_report.json`, skips already complete files, and only retries the missing ones on the next run.

Open:
- `~/px4-system-identification/hitl_runs/session_001/review/index.html`

HIL trajectory control:
- `TRJ_ACTIVE_ID = 100..104`
- `TRJ_MODE_CMD = 1` starts the selected trajectory
- `TRJ_MODE_CMD = 0` returns to position hold
- `TRJ_IDENT_PROF = 0..8` selects the identification profile
- `CST_POS_CTRL_TYP = 4` keeps the PX4 baseline, `6` switches to the identification path
