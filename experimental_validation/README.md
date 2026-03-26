Experimental Validation
=======================

This folder turns PX4 identification logs into a Gazebo-ready parameter candidate.

Main outputs
- total mass
- diagonal inertia terms
- drag summaries
- motor-model terms
- SDF comparison reports
- calibration-restore files from a QGroundControl dump
- paper-ready figures and CSVs

Required log types
- PX4 identification log
  - written by `trajectory_reader`
- Gazebo truth log
  - written by `SystemIdentificationLoggerPlugin`
  - used only in SITL studies

Important distinction
- `px4_only`
  - uses only the PX4 identification CSV
- `telemetry_augmented`
  - uses PX4 CSV plus telemetry-like Gazebo fields
- `truth_assisted`
  - uses the full Gazebo truth log

For real flights, only `px4_only` and telemetry-style fields are available.
`truth_assisted` is the simulator-side upper bound used to validate the method itself.

Current truth-assisted checkpoint
- Frozen candidate:
  - `examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/`
- Current comparable x500 SDF errors are effectively zero.
- Current blended twin score:
  - `99.99999999996612 / 100`

Why this matters
- If truth-assisted SITL does not recover the x500 SDF nearly exactly, the identification method is not ready.
- That upper-bound requirement is now satisfied.
- Any remaining future gap in real-flight use should therefore be treated as a logging / observability / sortie-design issue rather than a basic regression failure.

Run the estimator on one log pair
```bash
cd ~/px4-system-identification
python3 experimental_validation/cli.py \
  --csv /path/to/identification_log.csv \
  --truth-csv /path/to/gazebo_truth_log.csv \
  --ident-log \
  --out-dir experimental_validation/outputs/session_001
```

Generated files
- `identified_parameters.json`
- `candidate_inertial.sdf.xml`
- `candidate_vehicle_params.yaml`

Compare multiple sorties against the x500 SDF
```bash
cd ~/px4-system-identification
python3 experimental_validation/compare_with_sdf.py \
  --csv /path/to/mass_vertical.csv \
  --csv /path/to/hover_thrust.csv \
  --csv /path/to/roll_sweep.csv \
  --csv /path/to/pitch_sweep.csv \
  --csv /path/to/yaw_sweep.csv \
  --csv /path/to/drag_x.csv \
  --csv /path/to/drag_y.csv \
  --csv /path/to/drag_z.csv \
  --csv /path/to/motor_step.csv \
  --out-dir experimental_validation/outputs/x500_candidate
```

Generated comparison files
- `identified_parameters.json`
- `identified_parameters_by_mode.json`
- `sdf_reference.json`
- `sdf_comparison.json`
- `sdf_comparison_by_mode.json`
- `used_identification_logs.json`
- `candidate_x500_base.sdf`

Refresh paper assets from a completed SITL suite
```bash
cd ~/px4-system-identification
python3 experimental_validation/refresh_sitl_truth_artifacts.py \
  --results-root /path/to/completed_results_root \
  --candidate-name x500_truth_assisted_sitl_v1 \
  --out-dir examples/paper_assets
```

Generate paper-ready validation figures directly
```bash
cd ~/px4-system-identification
python3 experimental_validation/paper_artifacts.py \
  --candidate-json examples/paper_assets/candidates/x500_truth_assisted_sitl_v1/identified_parameters.json \
  --out-dir examples/paper_assets
```

What this produces
- five trajectory overlay figures:
  - `hairpin`, `lemniscate`, `circle`, `time_optimal_30s`, `minimum_snap_50s`
- five stress-test surfaces
- five stress-test slice plots
- direct parameter-error bars
- family-score bars
- trajectory-summary score chart
- CSV files and `paper_validation_summary.json`

Interpretation of the figures
- Stage 1 overlay figures:
  - use synthetic noisy stand-ins until real-flight logs are available
  - later you replace only the CSV inputs and regenerate the same plots
- Stage 2 stress-test surfaces and line plots:
  - keep the identified twin fixed
  - perturb the reference plant
  - quantify robustness against payload, COM shift, arm length, and motor-model mismatch
- Base-model-fit figures:
  - quantify how close the identified candidate is to the x500 SDF itself

QGroundControl parameter dump
- Put the latest exported parameter file here:
  - `experimental_validation/qgc/current_vehicle.params`

Restore calibration values after a firmware update
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore
```

Tests
```bash
cd ~/px4-system-identification
python3 -m unittest \
  experimental_validation.tests.test_estimators \
  experimental_validation.tests.test_identification_pipeline \
  experimental_validation.tests.test_sdf_compare \
  experimental_validation.tests.test_calibration_restore \
  experimental_validation.tests.test_composite_candidate \
  experimental_validation.tests.test_perfect_recovery_benchmark \
  experimental_validation.tests.test_paper_artifacts
```
