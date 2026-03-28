Experimental Validation
=======================

This folder turns PX4 and Gazebo logs into an identified multicopter model, compares that model against the x500 SDF, and regenerates the figure package used in the main README.

Default workspace assumptions
-----------------------------
- PX4 tree: `~/PX4-Autopilot-Identification`
- repository: `~/px4-system-identification`

CubeOrange build for hardware logging
-------------------------------------
If you want to run the same identification flow on CubeOrange hardware, first sync the overlay with the CubeOrange board file and build the hardware target:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification boards/cubepilot/cubeorange/default.px4board

cd ~/PX4-Autopilot-Identification
make cubepilot_cubeorange_default
```

Firmware artifact:
- `~/PX4-Autopilot-Identification/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`

Use this same firmware for the first USB-connected HIL/HITL check.

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
cd ~/px4-system-identification
python3 experimental_validation/build_latest_x500_candidate.py \
  --rootfs ~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs \
  --out-dir ~/px4-system-identification/experimental_validation/outputs/x500_candidate
```

This helper:
- picks the latest CSV for every required profile,
- reports missing profiles,
- finds the latest suitable truth log under `sysid_truth_logs`,
- writes the same comparison files as `compare_with_sdf.py`.

For `truth_assisted` comparisons, the intended operator workflow is one uninterrupted SITL session that contains all nine maneuver families. The helper is not meant to merge a single Gazebo truth file with profile logs collected across unrelated simulator restarts.

Use this helper instead of manual shell chains that assign `HOVER=...`, `ROLL=...`, and similar variables. Repeated maneuvers naturally create multiple CSV files. The helper is responsible for choosing the latest complete family.

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

Manual SITL note
----------------
The current repository version patches `x500/model.sdf` so `SystemIdentificationLoggerPlugin` is active in normal manual SITL sessions as well. Truth logs are written automatically into:
- `~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/sysid_truth_logs/`

If that folder is missing, resync the overlay and rebuild the dedicated PX4 workspace before trusting any comparison result.
If the folder exists but contains no CSV files after a completed maneuver, resync and restart anyway. An empty `sysid_truth_logs/` folder means the logger plugin was not active for that run.
Also avoid deleting the active truth CSV while SITL is still running. In older runs that left Gazebo writing into a deleted file descriptor. The current repository version reopens the file if it is removed, but deleting old truth logs before startup is still the cleanest workflow.

Common runtime note:
- `NodeShared::Publish() Error: Interrupted system call`
- `vehicle_imu ... timestamp error`

These are usually restart or transport interruptions around the end of a maneuver. Cleanly restart SITL, rerun the affected profile, and then rebuild the candidate with the helper above.

Smoke test
----------
```bash
cd ~/px4-system-identification
./examples/run_repo_smoke_test.sh
```
