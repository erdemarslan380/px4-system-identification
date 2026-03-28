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
make cubepilot_cubeorange_default upload
```

Build output:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

With the CubeOrange connected over USB, you can either upload directly from the terminal with `make cubepilot_cubeorange_default upload` or load the `.px4` file from QGroundControl.
If you use a different flight controller, change both the board file path and the `make <board>_default` / `make <board>_default upload` target together.

3. Build and open Gazebo SITL
-----------------------------
```bash
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

4. Generate and check the five validation trajectories
-----------------------------------------------------
The repository already ships five validation trajectories. Regenerate them with:

```bash
cd ~/px4-system-identification
python3 experimental_validation/validation_trajectories.py \
  --trajectories-dir ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories
```

Files created:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_100.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_101.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_102.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_103.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/id_104.traj`
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories/validation_manifest.json`

Trajectory map:
- `100`: `hairpin`, `28 s`
- `101`: `lemniscate`, `30 s`
- `102`: `circle`, `30 s`
- `103`: `time_optimal_30s`, `30 s`
- `104`: `minimum_snap_50s`, `50 s`

All five:
- start at the same point,
- start at `3 m` altitude,
- return to the same logged start pose.

5. Run the identification maneuvers in SITL
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

6. Run the five validation trajectories in SITL
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

7. HIL/HITL on CubeOrange with jMAVSim
-------------------------------------
Use jMAVSim only for the hardware-in-the-loop check. On this machine the verified cable split is:
- USB `ttyACM0`: jMAVSim serial link to the CubeOrange
- FTDI `ttyUSB0`: MAVLink shell or QGroundControl side link before the HIL run

Important:
- do not let QGroundControl and jMAVSim open the same `ttyACM0` device at the same time,
- if `ttyACM0` is reserved for jMAVSim, use the FTDI link for shell access and for the ground-station side connection.

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
- CubeOrange memory still below full use during the HIL boot path.

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

For trajectory checks in HIL, use the same `custom_pos_control` and `trajectory_reader` commands listed in the SITL sections of this README. Enter those board-side shell commands over the FTDI MAVLink console before launching jMAVSim, then keep QGroundControl and jMAVSim running for the actual HIL motion.

The intent is the same:
- baseline controller path: `custom_pos_control set px4_default`
- common start pose: `trajectory_reader abs_ref 0 0 -3 0`
- shared trajectory anchor: `trajectory_reader set_traj_anchor 0 0 -3`
- trajectory IDs: `100..104`

Do not expect motion from `trajectory_reader set_mode trajectory` on its own. The minimum trajectory command block is:

```bash
trajectory_reader set_mode position
trajectory_reader abs_ref 0 0 -3 0
trajectory_reader set_traj_anchor 0 0 -3
trajectory_reader set_traj_id 100
trajectory_reader set_mode trajectory
```

Open the board-side shell from the FTDI link with:

```bash
python3 ~/PX4-Autopilot-Identification/Tools/mavlink_shell.py /dev/ttyUSB0 -b 57600
```

Tracking CSV files for these runs are written under:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/`

7. Refresh the figure package
-----------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

Main outputs:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/`

8. Real-flight use
------------------
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

9. Current shipped results and figures
--------------------------------------
Current summary:
- blended twin score: `100.00 / 100`
- Stage-1 RMSE:
  - `hairpin`: stock `0.112 m`, twin `0.068 m`
  - `lemniscate`: stock `0.053 m`, twin `0.026 m`
  - `circle`: stock `0.068 m`, twin `0.031 m`
  - `time_optimal_30s`: stock `0.112 m`, twin `0.068 m`
  - `minimum_snap_50s`: stock `0.072 m`, twin `0.069 m`

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
