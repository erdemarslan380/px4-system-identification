Experimental Validation
=======================

This folder turns PX4 and Gazebo logs into an identified multicopter model, compares that model against the reference x500 SDF, and generates the figures used in the repository documentation.

Validation modes
----------------
- `px4_only`: use only PX4-side identification logs
- `telemetry_augmented`: use PX4 logs plus telemetry-like fields that are feasible in real flight
- `truth_assisted`: use the Gazebo truth logger and represent the simulator-side upper bound

The strongest current SITL claim is based on `truth_assisted`. That is the correct upper-bound check for method development.

Exact SITL log directories
--------------------------
- PX4 identification logs:
  - `~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/`
- PX4 tracking logs:
  - `~/PX4-Autopilot/build/px4_sitl_default/rootfs/tracking_logs/`
- Gazebo truth logs:
  - `~/PX4-Autopilot/build/px4_sitl_default/rootfs/sysid_truth_logs/`

Estimate one maneuver from the latest log pair
----------------------------------------------
```bash
LATEST_IDENT=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/*.csv | head -n 1)
LATEST_TRUTH=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/sysid_truth_logs/*.csv | head -n 1)

cd ~/px4-system-identification
python3 experimental_validation/cli.py \
  --csv "$LATEST_IDENT" \
  --truth-csv "$LATEST_TRUTH" \
  --ident-log \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/session_001
```

Generated files
---------------
- `identified_parameters.json`
- `candidate_inertial.sdf.xml`
- `candidate_vehicle_params.yaml`

Estimate a full x500 candidate from the whole maneuver family
-------------------------------------------------------------
```bash
HOVER=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/hover_thrust*.csv | head -n 1)
MASS=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/mass_vertical*.csv | head -n 1)
ROLL=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/roll_sweep*.csv | head -n 1)
PITCH=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/pitch_sweep*.csv | head -n 1)
YAW=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/yaw_sweep*.csv | head -n 1)
DRAG_X=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/drag_x*.csv | head -n 1)
DRAG_Y=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/drag_y*.csv | head -n 1)
DRAG_Z=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/drag_z*.csv | head -n 1)
MOTOR=$(ls -1t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/identification_logs/motor_step*.csv | head -n 1)

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

Generated comparison files
--------------------------
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/identified_parameters.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/identified_parameters_by_mode.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/sdf_reference.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/sdf_comparison.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/sdf_comparison_by_mode.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/used_identification_logs.json`
- `~/px4-system-identification/experimental_validation/outputs/x500_candidate/candidate_x500_base.sdf`

Generate the current full figure package
----------------------------------------

The recommended current path is the two-SITL placeholder bundle. It fills both Stage-1 slots now and lets you replace only the stock-side logs later.
```bash
cd ~/px4-system-identification
python3 experimental_validation/generate_sitl_validation_bundle.py \
  --mode placeholder \
  --candidate-dir ~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1 \
  --run-out-root ~/px4-system-identification/examples/paper_assets/stage1_inputs \
  --paper-assets-root ~/px4-system-identification/examples/paper_assets
```

This creates:
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/stock_sitl_proxy/`
- `~/px4-system-identification/examples/paper_assets/stage1_inputs/digital_twin_sitl/`
- `~/px4-system-identification/examples/paper_assets/paper_validation_summary.json`

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

This writes:
- five validation trajectory overlays with:
  - blue: stock x500 SITL proxy
  - orange: identified digital twin SITL
- five stress-test surfaces
- five stress-test line plots
- parameter error bars
- family score bars
- trajectory RMSE summary
- `paper_validation_summary.json`

Interpretation
--------------
- Stage 1 overlay plots are the place where real-flight traces will later be compared against the digital twin.
- In the current shipped repository state, the Stage-1 blue-side slot is populated by stock x500 SITL proxy logs with mild smooth perturbation.
- Stage 2 surface and line plots are robustness figures; they show how the fixed identified twin reacts when payload, COM, arm length, or motor dynamics are perturbed.
- The direct parameter-comparison files quantify how close the identified candidate is to the reference x500 SDF.

Current shipped upper-bound summary
-----------------------------------
- base blended twin score: `100.00 / 100`
- stage-1 stock proxy vs twin RMSE:
  - `hairpin`: `0.112 m` vs `0.068 m`
  - `lemniscate`: `0.053 m` vs `0.026 m`
  - `circle`: `0.068 m` vs `0.031 m`
  - `time_optimal_30s`: `0.112 m` vs `0.068 m`
  - `minimum_snap_50s`: `0.072 m` vs `0.069 m`

QGroundControl parameter dump
----------------------------
Put the latest exported parameter file here:
- `~/px4-system-identification/experimental_validation/qgc/current_vehicle.params`

Restore calibration values after a firmware update
--------------------------------------------------
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input ~/px4-system-identification/experimental_validation/qgc/current_vehicle.params \
  --out-dir ~/px4-system-identification/experimental_validation/qgc/restore
```

Tests
-----
```bash
cd ~/px4-system-identification
python3 -m unittest \
  experimental_validation.tests.test_estimators \
  experimental_validation.tests.test_identification_pipeline \
  experimental_validation.tests.test_sdf_compare \
  experimental_validation.tests.test_calibration_restore \
  experimental_validation.tests.test_composite_candidate \
  experimental_validation.tests.test_perfect_recovery_benchmark \
  experimental_validation.tests.test_paper_artifacts \
  experimental_validation.tests.test_repo_cleanliness
```
