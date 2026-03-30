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

6. Run one full campaign in SITL
--------------------------------
The repository now supports one built-in campaign:
- `identification_only`: 9 identification maneuvers
- `full_stack`: 9 identification maneuvers, then 5 validation trajectories

Return-to-anchor legs are handled inside `trajectory_reader` and are not logged.

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

Verified behavior in Gazebo SITL:
- the script climbs to the common `0 0 -3` hover point,
- `trajectory_reader` starts the campaign from a second terminal,
- `hover_thrust` opened one tracking CSV and one identification CSV,
- `mass_vertical` opened one additional tracking CSV and one additional identification CSV,
- the return leg between them did not create extra CSV files.

The full-stack campaign order is:
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

Logs appear in:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/`

For identification-only runs:
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

Build the candidate after the identification part finishes:
```bash
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

7. HIL/HITL smoke test on CubeOrange with jMAVSim
-------------------------------------------------
Use HIL only as a pre-flight smoke test:
- does one long campaign stay stable in one boot,
- do RAM and CPU stay healthy,
- does the SD card receive one CSV per maneuver and trajectory.

Keep the cable split simple:
- USB `ttyACM0`: jMAVSim
- QGroundControl: UDP only

During upload, close jMAVSim and QGroundControl:
```bash
cd ~/px4-system-identification
python3 examples/upload_cubeorange_firmware.py
```

Build jMAVSim once:
```bash
cd ~/PX4-Autopilot-Identification/Tools/simulation/jmavsim/jMAVSim
ant create_run_jar copy_res
```

Set the HIL airframe once in QGroundControl:
- `SYS_AUTOSTART = 1001`
- `SYS_HITL = 1`

Then start HIL:
```bash
cd ~/px4-system-identification
./examples/start_jmavsim_hitl.sh ~/PX4-Autopilot-Identification /dev/ttyACM0 921600
```

Open QGroundControl only in UDP mode.
The HIL airframe stages the hover reference at boot, so the smoke-test goal is one uninterrupted campaign in one boot.

Run the same campaign over UDP:
```bash
cd ~/px4-system-identification
python3 examples/run_mavlink_campaign.py \
  --endpoint udpin:127.0.0.1:14550 \
  --campaign full_stack \
  --timeout 520
```

After the run, the hardware check is simple:
- `9` identification CSV files under `/fs/microsd/identification_logs/`
- `14` tracking CSV files under `/fs/microsd/tracking_logs/`

That is the only HIL acceptance target for this repository. The five validation trajectory figures are produced from SITL and later real-flight logs, not from HIL.

9. Refresh the figure package
-----------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

Main outputs:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`

10. Off-Nominal SITL Check
--------------------------
This repository also ships a pre-HITL SITL methodology check that uses the five shipped trajectories exactly as provided:
- stock `x500` with baseline PX4 PID (`px4_default`),
- an off-nominal `x500_offnominal` with about `5%` mass and inertia reduction,
- slower motor dynamics and slightly altered motor coefficients,
- a light breeze world (`0.6 0.2 0.0 m/s`),
- the full nine identification maneuvers on the off-nominal model,
- re-identification against the perturbed SDF,
- a second five-trajectory pass in the windy off-nominal world.

Run it with:

```bash
cd ~/px4-system-identification
python3 experimental_validation/offnominal_sitl_study.py \
  --px4-root ~/PX4-Autopilot-Identification \
  --out-dir ~/px4-system-identification/examples/offnominal_sitl_study
```

Main outputs:
- `~/px4-system-identification/examples/offnominal_sitl_study/offnominal_study_summary.json`
- `~/px4-system-identification/examples/offnominal_sitl_study/candidate_offnominal_recovered/sdf_comparison.json`
- `~/px4-system-identification/examples/offnominal_sitl_study/figures/group_1_circle_hairpin_lemniscate.png`
- `~/px4-system-identification/examples/offnominal_sitl_study/figures/group_2_time_optimal_minimum_snap.png`

Current off-nominal identification result:
- recovered blended twin score: `100.00 / 100`
- recovered `mass`, `inertia`, `time constants`, `max rotor velocity`, and `motor constant` match the perturbed SDF in the current truth-assisted SITL check

Current trajectory RMSE summary:
- `circle`: stock `52.745 m`, off-nominal windy `55.425 m`
- `hairpin`: stock `109.208 m`, off-nominal windy `115.545 m`
- `lemniscate`: stock `84.210 m`, off-nominal windy `89.361 m`
- `time_optimal_30s`: stock `38.151 m`, off-nominal windy `39.947 m`
- `minimum_snap_50s`: stock `53.216 m`, off-nominal windy `55.833 m`

The requested legend naming is used in these figures:
- `Reference`
- `SITL`
- `Real flight results`

In this README section, `Real flight results` is only a label for the off-nominal windy SITL proxy so the visual style matches the planned real-flight comparison layout.

Three-trajectory panel:

![Off-Nominal Group 1](examples/offnominal_sitl_study/figures/group_1_circle_hairpin_lemniscate.png)

Two-trajectory panel:

![Off-Nominal Group 2](examples/offnominal_sitl_study/figures/group_2_time_optimal_minimum_snap.png)

11. Real-flight use
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

12. Review SD-card logs in an interactive 3D browser UI
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
- live USB CDC / MAVFTP pull: `pull_sdcard_logs_over_mavftp.py`

Before the live USB CDC pull:
- close `jMAVSim`,
- close `QGroundControl`,
- close any `mavlink_shell.py` session on `/dev/ttyACM0`.

`pull_sdcard_logs_over_mavftp.py` now:
- enforces single-process access to `/dev/ttyACM0`,
- skips files that are already complete locally,
- retries missing files on the next run,
- writes a transfer summary to `pull_report.json`.

To pull `.ulg` files from the PX4 log folder instead of the custom CSV folders:
```bash
cd ~/px4-system-identification
python3 examples/pull_sdcard_logs_over_mavftp.py \
  --port /dev/ttyACM0 \
  --baud 57600 \
  --destination-dir ~/px4-system-identification/hitl_runs/session_001 \
  --remote-group ulog_2026_03_28=/fs/microsd/log/2026-03-28 \
  --suffix .ulg
```

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

13. Current shipped results and figures
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
