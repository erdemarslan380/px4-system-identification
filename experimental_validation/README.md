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

Why a perfect SITL match is not automatic
- In a synthetic, perfectly informative benchmark, the estimator can recover the x500 SDF almost exactly.
- In practical Gazebo SITL sorties the vehicle is still controlled in closed loop by the PX4 baseline PID.
- That means the identification maneuvers excite the dynamics indirectly through the controller rather than by commanding the SDF parameters directly.
- Residual error therefore comes mainly from maneuver observability and family separation, not from measurement noise alone.

Current practical checkpoint
- The built-in practical candidate is `x500_family_composite_v2`.
- A frozen copy of the current candidate and its comparison report is stored at:
  - `examples/paper_assets/candidates/x500_family_composite_v2/`
- Current main x500 SDF errors are approximately:
  - mass: `+3.229%`
  - `Ixx`: `+24.458%`
  - `Iyy`: `-0.521%`
  - `Izz`: `+2.617%`
  - `time_constant_up`: `-10.556%`
  - `time_constant_down`: `0.000%`
- Current blended twin score: about `62.99 / 100`.

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
  experimental_validation.tests.test_composite_candidate \
  experimental_validation.tests.test_perfect_recovery_benchmark
```


Generate paper-ready validation figures
```bash
cd ~/px4-system-identification
python3 experimental_validation/paper_artifacts.py \
  --out-dir examples/paper_assets
```

What this produces
- synthetic placeholder real-flight overlays for five unseen validation trajectories
- stress-test surfaces for payload, center-of-mass shift, arm length, and motor-model variations
- CSV files and `paper_validation_summary.json` so the figures can be regenerated later with real-flight logs

Run the synthetic upper-bound benchmark
```bash
cd ~/px4-system-identification
python3 experimental_validation/perfect_recovery_benchmark.py \
  --out examples/paper_assets/perfect_recovery_benchmark.json
```

This benchmark answers a simple question:
- if the identification rows are perfectly informative and noiseless, does the estimator recover the x500 SDF?

The expected answer is yes, to numerical precision. Any remaining gap in the practical SITL pipeline is therefore a maneuver / observability issue, not a failure of the regression code itself.

If you already have a fresh identification output, point the script to it:
```bash
cd ~/px4-system-identification
python3 experimental_validation/paper_artifacts.py \
  --candidate-json /path/to/identified_parameters.json \
  --out-dir examples/paper_assets
```
