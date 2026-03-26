Experimental Validation
=======================

This folder turns PX4 identification logs into a Gazebo-ready parameter candidate.

Main outputs
- total mass
- thrust scale summary
- diagonal inertia terms
- drag coefficients
- motor-model terms
- SDF comparison reports
- calibration-restore files from a QGroundControl dump

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
`truth_assisted` is for simulator-side method development and validation.

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

Recommended sortie families
- hover and vertical excitation:
  - `hover_thrust`, `mass_vertical`
- inertia:
  - `roll_sweep`, `pitch_sweep`, `yaw_sweep`
- drag:
  - `drag_x`, `drag_y`, `drag_z`
- actuator dynamics:
  - `motor_step`

Tests
```bash
cd ~/px4-system-identification
python3 -m unittest \
  experimental_validation.tests.test_estimators \
  experimental_validation.tests.test_identification_pipeline \
  experimental_validation.tests.test_sdf_compare \
  experimental_validation.tests.test_calibration_restore \
  experimental_validation.tests.test_composite_candidate
```
