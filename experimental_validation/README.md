Experimental Validation
=======================

This folder turns PX4 and Gazebo logs into an identified multicopter model, compares that model against the x500 SDF, and regenerates the figure package used in the main README.

Default workspace assumptions
-----------------------------
- PX4 tree: `~/PX4-Autopilot-Identification`
- repository: `~/px4-system-identification`

Fastest way to refresh the shipped demo package
-----------------------------------------------
```bash
cd ~/px4-system-identification
./examples/refresh_demo_assets.sh ~/PX4-Autopilot-Identification
```

This command:
- regenerates the five validation trajectories,
- regenerates the current Stage-1 SITL proxy inputs,
- regenerates all shipped figures,
- updates `examples/paper_assets/paper_validation_summary.json`.

Main outputs
------------
- summary JSON:
  - `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`
- figures:
  - `~/px4-system-identification/examples/paper_assets/figures/`
- stage-1 inputs:
  - `~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy/`
  - `~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl/`

Estimate one maneuver
---------------------
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

Estimate the full x500 candidate
--------------------------------
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

Regenerate figures from an existing stock/twin bundle
-----------------------------------------------------
```bash
cd ~/px4-system-identification
python3 experimental_validation/sitl_validation_artifacts.py \
  --out-dir ~/px4-system-identification/examples/paper_assets \
  --stock-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy \
  --twin-root ~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl \
  --candidate-json ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/identified_parameters.json
```

Current shipped state
---------------------
- base blended twin score: `100.00 / 100`
- Stage-1 blue side currently uses stock x500 SITL proxy logs
- Later, replace only the stock-side CSV files with real-flight logs and rerun the figure command

Smoke test
----------
```bash
cd ~/px4-system-identification
./examples/run_repo_smoke_test.sh
```
