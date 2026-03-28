PX4 System Identification Workspace
==================================

This repository is for one workflow:
1. run system identification,
2. estimate an x500-compatible SDF candidate,
3. run five validation trajectories,
4. compare the digital twin with the reference traces.

Main folders
------------
- `overlay/`: PX4 and Gazebo additions
- `experimental_validation/`: estimation, comparison, figures
- `examples/`: short operator guides
- `system_identification.txt`: longer technical text for papers and reports

Fastest operator checklist:
- [operator_quickstart.md](/home/earsub/px4-system-identification/examples/operator_quickstart.md)

1. Create the dedicated PX4 workspace
------------------------------------
Use only this PX4 tree for this repository:
- `~/PX4-Autopilot-Identification`

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4-Autopilot-Identification
cd ~/PX4-Autopilot-Identification
bash ./Tools/setup/ubuntu.sh

cd ~
git clone git@github.com:erdemarslan380/px4-system-identification.git
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
```

2. Build for CubeOrange hardware
--------------------------------
For CubeOrange, resync once with the CubeOrange board file enabled:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/cubepilot/cubeorange/default.px4board

cd ~/PX4-Autopilot-Identification
make cubepilot_cubeorange_default
```

Build output:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

With the CubeOrange connected over USB, upload from the terminal with:

```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

This helper targets `/dev/ttyACM0` directly and uses a smaller CubeOrange-safe bootloader write size.
If you use a different flight controller, change both the board file path and the `make <board>_default` / `make <board>_default upload` target together.

3. Prepare the SD card for HITL and real flights
------------------------------------------------
For hardware runs, the trajectory files must be present on the SD card under `/fs/microsd/trajectories/`.

With the SD card mounted on the workstation:

```bash
cd ~/px4-system-identification
./examples/prepare_sdcard_payload.sh /media/$USER/<sdcard_mount_name>
```

This creates these directories on the card:
- `trajectories/`
- `tracking_logs/`
- `identification_logs/`

and copies the five shipped trajectory binaries:
- `id_100.traj`
- `id_101.traj`
- `id_102.traj`
- `id_103.traj`
- `id_104.traj`

Once the SD card is back on the CubeOrange, hardware logs are written to:
- `/fs/microsd/tracking_logs/`
- `/fs/microsd/identification_logs/`

The current overlay closes and flushes each CSV at the end of every run, so you can execute multiple identification profiles or trajectory runs in one boot session without rebooting PX4 between them.

4. Build and open Gazebo SITL
-----------------------------
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If this dedicated tree was created before the latest sync script update and the build stops with a missing `LTEST_MODE` parameter in `local_position_estimator`, resync once and rebuild from a clean SITL build directory:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
rm -rf ~/PX4-Autopilot-Identification/build/px4_sitl_default

cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If an older run is still open:
```bash
shutdown
pkill -f '/PX4-Autopilot-Identification/build/px4_sitl_default/bin/px4' || true
pkill -f 'gz sim' || true
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

5. Install and check the five shipped validation trajectories
-------------------------------------------------------------
This repository now uses only the shipped binary trajectory set under:
- `~/px4-system-identification/assets/validation_trajectories/`

Install those exact files into the PX4 workspace with:

```bash
cd ~/px4-system-identification
python3 experimental_validation/validation_trajectories.py \
  --trajectories-dir ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories
```

Files installed:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_100.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_101.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_102.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_103.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_104.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/validation_manifest.json`

Trajectory map:
- `100`: `hairpin`, `23 s`
- `101`: `lemniscate`, `19 s`
- `102`: `circle`, `15 s`
- `103`: `time_optimal_30s`, `11 s`
- `104`: `minimum_snap_50s`, `14 s`

The `.traj` binaries are used as-is. No new validation trajectories are generated by this repository.
All five trajectories start from the same anchored hover pose at `(0, 0, -3)` in local NED. They do not all end at the same point, so return to hover before launching the next one.

6. Run the identification maneuvers in SITL
-------------------------------------------
In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
trajectory_reader set_ident_profile hover_thrust
```

Then in QGroundControl:
1. arm,
2. take off manually to about `3 m`,
3. stabilize hover,
4. switch to `OFFBOARD`.

Run the full family in the same SITL session:
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

Wait for these PX4 messages before the next profile:
- `Identification maneuver completed: ...`
- `Identification log completed: ...`
- `Tracking log completed: ...`

Logs appear in:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/`

Build the candidate:
```bash
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

7. Run the five validation trajectories in SITL
-----------------------------------------------
Use the baseline PX4 controller path for the trajectory tests.

In `pxh>`:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

Then in QGroundControl:
1. arm,
2. take off manually to about `3 m`,
3. switch to `OFFBOARD`,
4. wait until the vehicle settles near the common start pose.

Run any trajectory with this pattern:
```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

After a trajectory finishes, return to the common start pose before the next one:
```bash
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
```

The five trajectory commands are:
```bash
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory

trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 101
trajectory_reader set_mode trajectory

trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 102
trajectory_reader set_mode trajectory

trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 103
trajectory_reader set_mode trajectory

trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 104
trajectory_reader set_mode trajectory
```

8. HIL/HITL on CubeOrange with jMAVSim
-------------------------------------
Use jMAVSim only for the hardware-in-the-loop check.

HITL cable layout for this repository:
- USB `ttyACM0`: jMAVSim serial link
- UDP: QGroundControl

Important:
- start jMAVSim before opening QGroundControl,
- in QGroundControl allow only `UDP` auto-connect during HITL,
- do not let QGroundControl and jMAVSim fight over the same serial device,
- close jMAVSim, QGroundControl, and any MAVLink shell before running a firmware upload,
- use the repository uploader helper for CubeOrange USB flashing.

Build jMAVSim once in the dedicated PX4 tree:

```bash
cd ~/PX4-Autopilot-Identification/Tools/simulation/jmavsim/jMAVSim
ant create_run_jar copy_res
```

Set the board to the HIL quadrotor airframe once in QGroundControl:
- `SYS_AUTOSTART = 1001`
- `SYS_HITL = 1`

This repository was checked with:
- `pwm_out_sim` active in HIL mode,
- `HIL_ACT_FUNC1..4 = 101..104`,
- CubeOrange flash use reduced to about `74.9%`,
- CubeOrange memory still below full use during the HIL boot path.

Before starting HITL, verify that the SD card already contains:
- `/fs/microsd/trajectories/id_100.traj`
- `/fs/microsd/trajectories/id_101.traj`
- `/fs/microsd/trajectories/id_102.traj`
- `/fs/microsd/trajectories/id_103.traj`
- `/fs/microsd/trajectories/id_104.traj`

From the workstation, the short pre-check is:

```bash
python3 ~/PX4-Autopilot-Identification/Tools/mavlink_shell.py /dev/ttyACM0 -b 57600
```

Then run:

```bash
ls /fs/microsd
ls /fs/microsd/trajectories
free
```

If the `trajectories/` directory is empty, `trajectory_reader` cannot start a hardware trajectory run.

Start jMAVSim on the USB link:

```bash
cd ~/px4-system-identification
./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification /dev/ttyACM0 921600
```

For a headless run:

```bash
cd ~/px4-system-identification
PX4_SYSID_HEADLESS=1 ./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification /dev/ttyACM0 921600
```

Open QGroundControl after jMAVSim starts and keep QGC in UDP mode only.

For trajectory checks in HIL, the complete order is:
1. connect CubeOrange over USB,
2. start jMAVSim on `ttyACM0`,
3. open QGroundControl and let it connect over UDP,
4. set the airframe once:
   - `SYS_AUTOSTART = 1001`
   - `SYS_HITL = 1`
5. the HIL airframe starts `custom_pos_control` and `trajectory_reader` automatically,
6. in QGroundControl, arm and take off to about `3 m`,
7. switch to `OFFBOARD`,
8. choose the active trajectory with `TRJ_ACTIVE_ID`,
9. set `TRJ_MODE_CMD = 1` to start the trajectory,
10. set `TRJ_MODE_CMD = 0` to return to position-hold.

Key HIL parameters:
- `CST_POS_CTRL_EN = 1`
- `CST_POS_CTRL_TYP = 4`
- `TRJ_ANCHOR_X = 0`
- `TRJ_ANCHOR_Y = 0`
- `TRJ_ANCHOR_Z = -3`
- `TRJ_ACTIVE_ID = 100..104`
- `TRJ_MODE_CMD = 0` for position hold, `1` for trajectory, `2` for identification

If you want to test identification instead of the five validation trajectories:
- set `CST_POS_CTRL_TYP = 6`
- set `TRJ_MODE_CMD = 2`
- set `TRJ_IDENT_PROF` to the desired profile index

During firmware upload, keep only the CubeOrange USB link active if possible. The reliable form is:

```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

Tracking CSV files for SITL/HITL trajectory runs are written under:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/` in SITL
- `/fs/microsd/tracking_logs/` on hardware

9. Refresh the figure package
-----------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

Main outputs:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`

10. Real-flight use
-------------------
Before the first real-flight test on CubeOrange, build and flash the hardware firmware:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/cubepilot/cubeorange/default.px4board

cd ~/PX4-Autopilot-Identification
make cubepilot_cubeorange_default
```

Flash this file from QGroundControl:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

Use the same two phases on hardware.

Phase A: identification
- build and flash your normal PX4 target with this overlay,
- take off manually to about `3 m`,
- switch to `OFFBOARD`,
- run the same nine `set_ident_profile ...` commands,
- copy the logs from:
  - `/fs/microsd/identification_logs/`
  - `/fs/microsd/tracking_logs/`

Phase B: trajectory validation
- keep `custom_pos_control` in `px4_default`,
- take off manually to about `3 m`,
- switch to `OFFBOARD`,
- move to the common start pose,
- run the same five trajectory IDs:
  - `100 hairpin`
  - `101 lemniscate`
  - `102 circle`
  - `103 time_optimal_30s`
  - `104 minimum_snap_50s`
- copy the resulting tracking CSV files from `/fs/microsd/tracking_logs/`

The real-flight command pattern is the same as SITL:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control set px4_default
custom_pos_control enable
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

A slightly longer sortie plan is here:
- [real_flight_sorties.md](/home/earsub/px4-system-identification/examples/real_flight_sorties.md)

11. Review SD-card logs in an interactive 3D browser UI
-------------------------------------------------------
After a HITL or real-flight session, copy the SD-card logs into the repository and build the review bundle:

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

Use only one import path per session:
- mounted SD card: `import_sdcard_logs.sh`
- live FTDI/MAVLink pull: `pull_sdcard_logs_over_mavftp.py`

Open:
- `~/px4-system-identification/hitl_runs/session_001/review/index.html`

The review page provides:
- a zoomable and pannable 3D path viewer,
- reference and vehicle overlays when both are present,
- X/Y/Z versus time plots,
- per-run duration, row count, and position RMSE,
- direct links to the copied raw CSV files.

To keep the inspection package in GitHub, commit both the imported CSV folders and the generated `review/` folder.

The repository also ships a demo review bundle generated from the current SITL tracking logs:
- [HITL Review Demo](/home/earsub/px4-system-identification/examples/hitl_review_demo/index.html)

12. Current shipped results and figures
---------------------------------------
Current summary:
- blended twin score: `100.00 / 100`
- Stage-1 RMSE:
  - `hairpin`: stock `0.451 m`, twin `0.150 m`
  - `lemniscate`: stock `0.336 m`, twin `0.113 m`
  - `circle`: stock `0.438 m`, twin `0.111 m`
  - `time_optimal_30s`: stock `0.644 m`, twin `0.450 m`
  - `minimum_snap_50s`: stock `0.597 m`, twin `0.341 m`

Summary figure:

![Trajectory RMSE Summary](examples/paper_assets/figures/trajectory_rmse_summary.png)

Parameter summary:

![Parameter Error Bars](examples/paper_assets/figures/parameter_error_bars.png)

Trajectory overlays:

![Hairpin Overlay](examples/paper_assets/figures/hairpin_overlay.png)
![Lemniscate Overlay](examples/paper_assets/figures/lemniscate_overlay.png)
![Circle Overlay](examples/paper_assets/figures/circle_overlay.png)
![Time Optimal Overlay](examples/paper_assets/figures/time_optimal_30s_overlay.png)
![Minimum Snap Overlay](examples/paper_assets/figures/minimum_snap_50s_overlay.png)

More figures and the full summary JSON are in:
- `~/px4-system-identification/examples/paper_assets/`
