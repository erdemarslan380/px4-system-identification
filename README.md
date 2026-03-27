PX4 System Identification Workspace
==================================

This repository is a standalone workspace for one focused job: recover multicopter model parameters from PX4 logs and turn them into a Gazebo digital twin.

It is written for control researchers. The intended workflow is:
1. apply the overlay into an upstream PX4 tree,
2. build and run Gazebo SITL,
3. execute the built-in identification maneuvers,
4. estimate SDF parameters from the recorded logs,
5. validate the identified model with five reference trajectories,
6. repeat the same process later with real-flight logs.

What is in this repository
- `overlay/`: the PX4 and Gazebo source overlay used during build
- `experimental_validation/`: the offline parameter-estimation and validation scripts
- `examples/`: operator walkthroughs, sortie definitions, and generated figures
- `system_identification.txt`: long-form method description for papers and reports
- `sync_into_px4_workspace.sh`: copies the overlay into an upstream PX4 workspace

What is not in this repository
- the full PX4 source tree
- QGroundControl
- Gazebo itself
- the separate optimization / planner / dashboard stack

1. Installation
---------------

Use one dedicated PX4 tree for this repository:
- `~/PX4-Autopilot-Identification`

Do not use a different PX4 workspace for this workflow.

Commands:
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4-Autopilot-Identification
cd ~/PX4-Autopilot-Identification
bash ./Tools/setup/ubuntu.sh

cd ~
git clone git@github.com:erdemarslan380/px4-system-identification.git
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification

cd ~/PX4-Autopilot-Identification
make px4_sitl gz_x500
```

Healthy first-build signs:
- `Generating uORB topic headers`
- `Linking CXX executable bin/px4`
- `INFO [init] Gazebo simulator ...`
- `INFO [init] Starting gazebo with world: ...`
- `pxh>`

Benign warnings during the first run:
- the CMake `CMP0148` warning
- stock x500 `gz_frame_id` warnings
- temporary `No connection to the GCS` preflight warnings before QGroundControl connects

2. SITL Identification Test
---------------------------

Open Gazebo visually:
```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

If Gazebo server starts but the GUI window does not appear, open it in another terminal:
```bash
gz sim -g
```

Start the identification helper modules in the PX4 shell:
```bash
custom_pos_control start
trajectory_reader start
custom_pos_control enable
custom_pos_control set sysid
trajectory_reader set_mode identification
trajectory_reader set_ident_profile hover_thrust
```

Then use your normal safe bootstrap:
1. connect QGroundControl,
2. arm,
3. take off manually to about `3 m`,
4. stabilize the hover,
5. switch to `OFFBOARD`.

On the current build, `trajectory_reader` keeps the selected identification mode after OFFBOARD entry. When the maneuver starts, PX4 now prints all key states directly in the console:
- `Identification maneuver started: ...`
- `Purpose: ...`
- `Estimated duration: ...`
- `Identification maneuver completed: ...`
- `Identification log completed: ...`
- `Tracking log completed: ...`

That is the signal to wait for before sending the next profile command.

Identification maneuvers
------------------------

| Profile | What it excites | Approx. duration | What happens after completion |
| --- | --- | ---: | --- |
| `hover_thrust` | hover-thrust tracking around a constant hover point | `26 s` | the vehicle holds the final reference |
| `mass_vertical` | multi-frequency vertical motion for mass and thrust scale | `36 s` | the vehicle holds the final reference |
| `roll_sweep` | lateral motion for roll-axis inertia and coupling | `28 s` | the vehicle holds the final reference |
| `pitch_sweep` | longitudinal motion for pitch-axis inertia and coupling | `28 s` | the vehicle holds the final reference |
| `yaw_sweep` | yaw excitation for yaw inertia and yaw moment balance | `24 s` | the vehicle holds the final reference |
| `drag_x` | forward-back motion for X-axis drag | `30 s` | the vehicle holds the final reference |
| `drag_y` | side-to-side motion for Y-axis drag | `30 s` | the vehicle holds the final reference |
| `drag_z` | vertical motion for Z-axis drag | `30 s` | the vehicle holds the final reference |
| `motor_step` | step-like thrust sequence for motor time constants | `24 s` | the vehicle holds the final reference |

Run one profile at a time, and wait for the completion messages before moving on:
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

Exact SITL log directories
--------------------------

All SITL logs are written into the PX4 rootfs under the build tree:
- PX4 identification logs:
  - `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/`
- PX4 tracking logs:
  - `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/tracking_logs/`
- Gazebo truth logs:
  - `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/`

3. Estimate the SDF Parameters
------------------------------

Estimate from the latest SITL log pair:
```bash
LATEST_IDENT=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/*.csv | head -n 1)
LATEST_TRUTH=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/*.csv | head -n 1)

cd ~/px4-system-identification
python3 experimental_validation/cli.py \
  --csv "$LATEST_IDENT" \
  --truth-csv "$LATEST_TRUTH" \
  --ident-log \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/session_001
```

Main outputs:
- `~/px4-system-identification/experimental_validation/outputs/session_001/identified_parameters.json`
- `~/px4-system-identification/experimental_validation/outputs/session_001/candidate_inertial.sdf.xml`
- `~/px4-system-identification/experimental_validation/outputs/session_001/candidate_vehicle_params.yaml`

Build a combined candidate from the full maneuver family:
```bash
HOVER=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/hover_thrust*.csv | head -n 1)
MASS=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/mass_vertical*.csv | head -n 1)
ROLL=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/roll_sweep*.csv | head -n 1)
PITCH=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/pitch_sweep*.csv | head -n 1)
YAW=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/yaw_sweep*.csv | head -n 1)
DRAG_X=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/drag_x*.csv | head -n 1)
DRAG_Y=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/drag_y*.csv | head -n 1)
DRAG_Z=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/drag_z*.csv | head -n 1)
MOTOR=$(ls -1t ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/identification_logs/motor_step*.csv | head -n 1)

cd ~/px4-system-identification
python3 experimental_validation/compare_with_sdf.py \
  --csv "$MASS" \
  --csv "$HOVER" \
  --csv "$ROLL" \
  --csv "$PITCH" \
  --csv "$YAW" \
  --csv "$DRAG_X" \
  --csv "$DRAG_Y" \
  --csv "$DRAG_Z" \
  --csv "$MOTOR" \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

Comparison outputs:
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/identified_parameters.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/sdf_comparison.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/sdf_comparison_by_mode.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/candidate_x500_base.sdf`

4. Build the Current SITL Comparison Package
--------------------------------------------

Until real-flight logs exist, this repository fills both Stage-1 comparison slots from SITL:
- blue: `stock x500 SITL proxy`
- orange: `identified digital twin SITL`

The blue side includes a small smooth perturbation so the two traces are not visually identical. Later you only need to replace the blue-side CSVs with real-flight logs and rerun the same artifact command.

Generate the current full paper appendix bundle:
```bash
cd ~/px4-system-identification
python3 experimental_validation/generate_sitl_validation_bundle.py \
  --mode placeholder \
  --candidate-dir ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1 \
  --run-out-root ~/px4-system-identification/examples/paper_assets/stage1_inputs \
  --paper-assets-root ~/px4-system-identification/examples/paper_assets
```

This writes the exact Stage-1 input roots here:
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy/`
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl/`

The generated figures and summary land here:
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- `~/px4-system-identification/examples/paper_assets/figures/parameter_error_bars.png`
- `~/px4-system-identification/examples/paper_assets/figures/family_score_bars.png`
- `~/px4-system-identification/examples/paper_assets/figures/trajectory_rmse_summary.png`
- `~/px4-system-identification/examples/paper_assets/figures/hairpin_overlay.png`
- `~/px4-system-identification/examples/paper_assets/figures/lemniscate_overlay.png`
- `~/px4-system-identification/examples/paper_assets/figures/circle_overlay.png`
- `~/px4-system-identification/examples/paper_assets/figures/time_optimal_30s_overlay.png`
- `~/px4-system-identification/examples/paper_assets/figures/minimum_snap_50s_overlay.png`
- five stress-test surfaces
- five stress-test line plots

Current shipped summary with the truth-assisted upper-bound candidate:
- base blended twin score: `100.00 / 100`
- family scores: inertia `100.00`, mass `100.00`, motor coefficients `100.00`, motor dynamics `100.00`
- Stage-1 RMSE, stock proxy vs twin:
  - `hairpin`: `0.112 m` vs `0.068 m`
  - `lemniscate`: `0.053 m` vs `0.026 m`
  - `circle`: `0.068 m` vs `0.031 m`
  - `time_optimal_30s`: `0.112 m` vs `0.068 m`
  - `minimum_snap_50s`: `0.072 m` vs `0.069 m`

Later, when real-flight logs exist, keep the orange twin side and replace the blue-side root with your real-flight tracking bundle:
```bash
cd ~/px4-system-identification
python3 experimental_validation/sitl_validation_artifacts.py \
  --out-dir ~/px4-system-identification/examples/paper_assets \
  --stock-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy \
  --twin-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl \
  --candidate-json ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/identified_parameters.json
```

5. Hardware Build and Flash
---------------------------

Sync the same overlay into the PX4 tree before a hardware build:
```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/px4/fmu-v3/default.px4board
```

Then build your firmware target from the PX4 tree in the normal PX4 way. The exact board target depends on your flight controller. The important point is that the overlay must already be synced before you build.

Calibration restore after a firmware update
-------------------------------------------

Export all parameters from QGroundControl and place the file here:
- `~/px4-system-identification/experimental_validation/qgc/current_vehicle.params`

Then generate the restore files:
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input ~/px4-system-identification/experimental_validation/qgc/current_vehicle.params \
  --out-dir ~/px4-system-identification/experimental_validation/qgc/restore
```

6. Real-Flight Identification Sorties
-------------------------------------

The real-flight baseline is the PX4 default controller path. The identification maneuvers are still triggered through the overlay, but the aircraft should be flown conservatively:
1. start the helper modules before switching to OFFBOARD,
2. take off manually,
3. stabilize around `3 m`,
4. switch to OFFBOARD only after the hover is healthy,
5. run one profile,
6. wait for the completion messages,
7. recover to hover,
8. start the next profile.

Recommended sortie structure:
- Sortie 1: `hover_thrust`, `mass_vertical`
- Sortie 2: `roll_sweep`, `pitch_sweep`
- Sortie 3: `yaw_sweep`, `motor_step`
- Sortie 4: `drag_x`, `drag_y`, `drag_z`
- Sortie 5: the five validation trajectories if battery budget allows, otherwise split across more batteries

A detailed operator sequence is in:
- `~/px4-system-identification/examples/real_flight_sorties.md`

Typical hardware-side log folders
---------------------------------

On NuttX targets, `PX4_STORAGEDIR` is typically backed by the SD card. In practice you should expect these folders on the vehicle storage:
- `/fs/microsd/identification_logs/`
- `/fs/microsd/tracking_logs/`

After each field session, copy those CSV files into a concrete session folder such as:
- `~/px4-system-identification/experimental_validation/outputs/real_flights/2026-03-27_session_01/raw_logs/`

7. Write the Identified Model into an SDF
-----------------------------------------

The estimator writes an SDF-ready inertial snippet at:
- `candidate_inertial.sdf.xml`

The repository also provides a direct helper that prepares a complete Gazebo model:
```bash
cd ~/px4-system-identification
python3 experimental_validation/prepare_identified_model.py \
  --px4-root ~/PX4-Autopilot-Identification \
  --candidate-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate \
  --model-name x500_identified
```

This creates:
- `~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_identified/`
- `~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_identified_base/`

The stock x500 model remains untouched for side-by-side comparison.

8. Five Validation Trajectories
-------------------------------

This repository uses five validation trajectories:
- `hairpin`
- `lemniscate`
- `circle`
- `time_optimal_30s`
- `minimum_snap_50s`

They are intentionally defined so that the logged mission segment:
- starts from the same XY point,
- starts at the same safe altitude (`3 m` in NED, i.e. `z = -3`),
- returns to the same logged start pose,
- can be overlaid directly across real flight and digital-twin simulation.

Operationally, both in SITL and in real flight, the vehicle should:
1. take off and stabilize at about `3 m`,
2. move to the common trajectory start pose,
3. begin logging only after that start pose is reached,
4. execute the trajectory,
5. return to hover.

9. Simulate the Five Trajectories with the New SDF
--------------------------------------------------

Once the identified SDF candidate is in Gazebo, use the prepared five-trajectory bundle as the comparison backbone. The default Stage-1 input roots remain:
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy/`
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl/`

You can rerun the full figure package at any time with:
```bash
cd ~/px4-system-identification
python3 experimental_validation/generate_sitl_validation_bundle.py \
  --mode placeholder \
  --candidate-dir ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1 \
  --run-out-root ~/px4-system-identification/examples/paper_assets/stage1_inputs \
  --paper-assets-root ~/px4-system-identification/examples/paper_assets
```

10. Fly the Same Five Trajectories in Real Flight
-------------------------------------------------

Use the exact same common start pose and altitude policy as in SITL:
- manual takeoff to `3 m`
- hover stabilization
- OFFBOARD entry
- move to the common start point
- start logging
- execute one trajectory
- return to hover
- continue only if battery and safety margins are still healthy

11. Plot Real Flight Versus Digital Twin
----------------------------------------

After the real-flight logs are copied into the repository, regenerate the overlay figures so each trajectory shows:
- the commanded reference mission,
- the real-flight trace,
- the digital-twin trace,
- error-norm curves over time.

The simplest drop-in workflow is:
1. replace the five blue-side CSVs under:
   - `~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy/tracking_logs/`
2. keep the file names:
   - `hairpin.csv`
   - `lemniscate.csv`
   - `circle.csv`
   - `time_optimal_30s.csv`
   - `minimum_snap_50s.csv`
3. rerun:
```bash
cd ~/px4-system-identification
python3 experimental_validation/sitl_validation_artifacts.py \
  --out-dir ~/px4-system-identification/examples/paper_assets \
  --stock-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy \
  --twin-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl \
  --candidate-json ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/identified_parameters.json
```

Those refreshed overlays live in:
- `~/px4-system-identification/examples/paper_assets/figures/`

Current honest claim
--------------------

At the moment, the strongest claim this repository supports is:
- the truth-assisted SITL upper bound has been solved to numerical precision,
- the full figure-generation pipeline is working,
- the maneuver families, logging structure, and post-processing flow are fixed,
- the current shipped Stage-1 blue-side traces are stock x500 SITL proxy logs,
- real-flight equivalence still requires replacing those blue-side Stage-1 logs with actual outdoor logs.

Additional detailed references
------------------------------
- long-form method description:
  - `~/px4-system-identification/system_identification.txt`
- manual visual Gazebo walkthrough:
  - `~/px4-system-identification/examples/visual_sitl_walkthrough.md`
- real-flight sortie checklist:
  - `~/px4-system-identification/examples/real_flight_sorties.md`
- offline estimation guide:
  - `~/px4-system-identification/experimental_validation/README.md`
