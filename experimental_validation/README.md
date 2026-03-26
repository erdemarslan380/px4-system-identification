# Experimental Validation

This folder estimates Gazebo SDF parameters from PX4 identification flights.

The usual workflow is:
1. run a `sysid` task from the optimization toolkit
2. collect the exported identification CSV
3. estimate the vehicle parameters
4. copy the result into an SDF model

## Current outputs

The estimator can produce:
- total mass
- thrust scale
- diagonal inertia terms
- simple quadratic drag coefficients
- SDF reference comparison for the identifiable subset
- calibration-restore files from a QGroundControl dump

## QGroundControl parameter dump

If you export all vehicle parameters from QGroundControl, place the file here:

- `experimental_validation/qgc/current_vehicle.params`

The optimization toolkit can pass this file into identification runs with the task field:
- `base_param_file`

## PX4 identification log mode

The new `sysid` controller and `identification` mission mode write CSV logs under:

- `build/px4_sitl_default/rootfs/identification_logs/`

Finished runs also archive them inside:

- `Tools/optimization/plan_runs/<study>/<task>/identification_traces/`

## Supported identification motions

- `hover_thrust`
- `mass_vertical`
- `roll_sweep`
- `pitch_sweep`
- `yaw_sweep`
- `drag_x`
- `drag_y`
- `drag_z`
- `motor_step`

Use several motions if you want a full SDF estimate. A hover-only run is now good enough to validate logging plus the motor-model subset, but inertia, drag, and actuator timing still need the dedicated sweep profiles.

## Run the comprehensive x500 identification suite

```bash
cd ~/px4-custom
python3 Tools/optimization/run_simulation_plan.py \
  --plan Tools/optimization/generated_plans/x500_identification_comprehensive.yaml \
  --clean \
  --serve-dashboard
```

This plan repeats the most informative motions so the estimator can reject transient segments and compare the recovered parameters against the x500 SDF.

## Run the estimator on a PX4 identification log

```bash
cd ~/px4-custom
python3 experimental_validation/cli.py \
  --csv Tools/optimization/plan_runs/<study>/<task>/identification_traces/eval_00000.csv \
  --ident-log \
  --out-dir experimental_validation/outputs/session_001
```

Generated outputs:
- `identified_parameters.json`
- `candidate_inertial.sdf.xml`
- `candidate_vehicle_params.yaml`

## Compare the identified result against the x500 SDF

```bash
cd ~/px4-custom
python3 experimental_validation/compare_with_sdf.py \
  --results-root Tools/optimization/plan_runs/x500_identification_suite \
  --out-dir experimental_validation/outputs/x500_identification_suite
```

Generated outputs:
- `sdf_reference.json`
- `sdf_comparison.json`
- `candidate_x500_base.sdf`

## Restore calibration values after a firmware update

```bash
cd ~/px4-custom
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore
```

Generated outputs:
- `selected_calibration_params.json`
- `restore_calibration.params`
- `restore_calibration.nsh`

## Tests

```bash
python3 -m unittest \
  experimental_validation.tests.test_estimators \
  experimental_validation.tests.test_identification_pipeline \
  experimental_validation.tests.test_sdf_compare \
  experimental_validation.tests.test_calibration_restore
```
