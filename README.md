PX4 SITL Workflow Guide
=======================

Scope
-----
This README is the repeatable `SITL` workflow for this repo.

The goal is that someone can start from a clean terminal and run the same steps
we are using now:

1. clean up all old `PX4/Gazebo/QGC/helper` leftovers
2. take one visible stock `x500 + circle` smoke test
3. collect the full `5/5` stock validation set
4. prepare the `jMAVSim`-prior SDF and collect its `5/5` set
5. run the `9/9` SITL identification suite on that prior model
6. build a new candidate from those ident logs
7. collect the `5/5` validation set for the re-identified model
8. draw the final three-model comparison figures

Fixed project rules
-------------------
These are intentionally stable and should not be changed between tests:

- every new top-level SITL test starts with cleanup
- the SITL flight flow is fixed:
  `arm -> takeoff -> hover -> offboard -> modules -> trajectory/ident -> POSCTL -> land`
- `SIM_GZ_EC_MIN/MAX` is not tuned per test
- if a changed SDF flies poorly under the fixed SITL mapping, we fix the model, not the run command
- trajectory figures show raw `RMSE` against the reference trajectory in the title
- color maps in the figures still show along-path shape error

Workspace
---------
Use these two repos together:

- `~/PX4-Autopilot-Identification`
- `~/px4-system-identification`

If needed:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
```

Clean start before every new SITL test
--------------------------------------
Run this before every new top-level test or suite:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh
```

Optional best-effort USB reset for a specific device:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh --device /dev/ttyACM0 --reset-usb
```

What the cleanup does:

- closes visible `PX4 Gazebo Nested Display` and console windows
- stops stale `PX4 SITL`, `Gazebo`, console tail, helper, and old runner processes
- releases common serial ports used by `HITL/QGC`
- does not intentionally close browser windows such as `Firefox`
- if you stop a visual run with `Ctrl+C`, the runner now also triggers the same background cleanup on exit; if a second launch still behaves oddly, run the cleanup script once manually and start again

Visible stock smoke test
------------------------
Use this first after a reboot or after any large change.

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/run_sitl_validation.py \
  --trajectory-names circle \
  --model-labels stock_sitl_placeholder \
  --visual \
  --show-console \
  --out-root /tmp/sitl_stock_circle_visual
```

Expected windows:

- `PX4 Gazebo Nested Display`
- `PX4 SITL Log (read-only)`

Notes:

- the console window is a live log viewer, not an interactive `pxh>` shell
- visual mode uses the fixed near top-down follow view
- the runner already applies the required no-RC/no-GCS SITL preflight settings
- this is the first command to use when you want to watch the motion in Gazebo
- the runner first tries `PX4 Gazebo Nested Display`; if that window cannot become visible, it falls back to the normal host `Gazebo Sim` window

Current camera default:

- `PX4_GZ_FOLLOW_OFFSET_X=0`
- `PX4_GZ_FOLLOW_OFFSET_Y=0`
- `PX4_GZ_FOLLOW_OFFSET_Z=7`

If you need a real interactive `pxh>` shell instead of the read-only log window:

```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

Stock 5-trajectory validation set
---------------------------------
This collects one tracking CSV per validation trajectory for the stock `x500`.

The default command below is headless. That is why it can look like it is
"waiting" in the terminal while the five trajectories are being executed one by
one in the background.

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label stock_sitl_placeholder \
  --out-root /tmp/sitl_stock_suite
```

If you want to watch all five stock trajectories in Gazebo instead of running
headless, use the same collector with `--visual --show-console`:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label stock_sitl_placeholder \
  --visual \
  --show-console \
  --out-root /tmp/sitl_stock_suite_visual
```

Outputs:

- dataset root:
  `/tmp/sitl_stock_suite/stock_sitl_placeholder`
- tracking logs:
  `/tmp/sitl_stock_suite/stock_sitl_placeholder/tracking_logs/*.csv`
- summary:
  `/tmp/sitl_stock_suite/stock_sitl_placeholder/collection_summary.json`

Stock-only figures and interactive review:

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --out-dir /tmp/sitl_stock_suite/stock_only_figures

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --out-dir /tmp/sitl_stock_suite/stock_only_review
```

Review outputs:

- static PNGs:
  `/tmp/sitl_stock_suite/stock_only_figures/group_1_circle_hairpin_lemniscate.png`
- static PNGs:
  `/tmp/sitl_stock_suite/stock_only_figures/group_2_time_optimal_minimum_snap.png`
- interactive 3D HTML:
  `/tmp/sitl_stock_suite/stock_only_review/index.html`

Open the HTML in Firefox if you want zoom, pan, and rotate inspection per
trajectory.

Prepare the jMAVSim-prior SDF
-----------------------------
This installs a prepared Gazebo model inside the PX4 tree.

```bash
cd ~/px4-system-identification

python3 experimental_validation/prepare_identified_model.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-name x500_ident_matrix_prior
```

Installed model paths:

- `~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior`
- `~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior_base`

jMAVSim prior parameter table
-----------------------------
These are the fixed prior values transferred from the local PX4 `jMAVSim`
defaults into the Gazebo candidate:

| Parameter | Value | Unit |
|---|---:|---:|
| Mass | 0.8 | kg |
| Thrust scale | 16.0 | N/cmd |
| Inertia X | 0.005 | kg*m^2 |
| Inertia Y | 0.005 | kg*m^2 |
| Inertia Z | 0.009 | kg*m^2 |
| Drag X | 0.01 | - |
| Drag Y | 0.01 | - |
| Drag Z | 0.01 | - |
| Time constant up | 0.005 | s |
| Time constant down | 0.005 | s |
| Max rotor velocity | 1000.0 | rad/s |
| Motor constant | 4.0e-06 | N/(rad/s)^2 |
| Moment constant | 0.0125 | - |
| Rotor drag coefficient | 8.06428e-05 | - |
| Rolling moment coefficient | 1.0e-06 | - |
| Rotor velocity slowdown | 10.0 | - |

If you want the same table as a saved markdown/json artifact:

```bash
cd ~/px4-system-identification

python3 experimental_validation/summarize_candidate_parameters.py \
  --candidate-a examples/paper_assets/candidates/jmavsim_prior_v1 \
  --label-a "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_parameters
```

jMAVSim-prior 5-trajectory validation set
-----------------------------------------
The dataset collector now supports custom model labels and Gazebo model names.

Visible `jMAVSim`-prior smoke test:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label jmavsim_prior_sitl \
  --gz-model x500_ident_matrix_prior \
  --display-name "jMAVSim prior SDF" \
  --trajectory-names circle \
  --visual \
  --show-console \
  --out-root /tmp/sitl_jmavsim_prior_visual
```

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label jmavsim_prior_sitl \
  --gz-model x500_ident_matrix_prior \
  --display-name "jMAVSim prior SDF" \
  --out-root /tmp/sitl_jmavsim_prior_suite
```

Outputs:

- dataset root:
  `/tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl`
- tracking logs:
  `/tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl/tracking_logs/*.csv`

Stock vs jMAVSim-prior figures and interactive review:

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --compare-root /tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_suite/stock_vs_prior_figures

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --compare-root /tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_suite/stock_vs_prior_review
```

Review outputs:

- static PNGs:
  `/tmp/sitl_jmavsim_prior_suite/stock_vs_prior_figures/group_1_circle_hairpin_lemniscate.png`
- static PNGs:
  `/tmp/sitl_jmavsim_prior_suite/stock_vs_prior_figures/group_2_time_optimal_minimum_snap.png`
- interactive 3D HTML:
  `/tmp/sitl_jmavsim_prior_suite/stock_vs_prior_review/index.html`

jMAVSim-prior 9-profile SITL identification suite
-------------------------------------------------
This runs the same fixed SITL flight flow, but switches the payload phase from
validation trajectories to the nine built-in identification profiles.

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/run_sitl_ident_suite.py \
  --out-root /tmp/sitl_ident_jmavsim_prior \
  --label jmavsim_prior_ident_suite \
  --model-name x500_ident_matrix_prior \
  --source-model-dir ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior
```

Outputs:

- manifest:
  `/tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite/run_manifest.json`
- per-profile ident logs:
  `/tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite/*/identification_traces/eval_00000.csv`
- per-profile tracking logs:
  `/tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite/*/tracking_logs/run_00000.csv`
- per-profile truth logs:
  `/tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite/*/gazebo_truth_traces/eval_00000.csv`

Interactive ident review bundle:

```bash
cd ~/px4-system-identification

python3 experimental_validation/build_hitl_review_bundle.py \
  --log-root /tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite \
  --out-dir /tmp/sitl_ident_jmavsim_prior/review
```

Review output:

- interactive HTML:
  `/tmp/sitl_ident_jmavsim_prior/review/index.html`

Build a new candidate from the SITL ident logs
----------------------------------------------
This uses the existing ident code path. The reference SDF is the prepared
`jMAVSim`-prior model that was actually flown during the ident suite.

```bash
cd ~/px4-system-identification

python3 experimental_validation/compare_with_sdf.py \
  --results-root /tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite \
  --out-dir /tmp/sitl_reidentified_candidate \
  --sdf-model ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior/model.sdf \
  --sdf-base-model ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior_base/model.sdf
```

Outputs:

- candidate parameters:
  `/tmp/sitl_reidentified_candidate/identified_parameters.json`
- patched base SDF:
  `/tmp/sitl_reidentified_candidate/candidate_x500_base.sdf`
- comparison report:
  `/tmp/sitl_reidentified_candidate/sdf_comparison.json`

jMAVSim prior vs re-identified parameter table:

```bash
cd ~/px4-system-identification

python3 experimental_validation/summarize_candidate_parameters.py \
  --candidate-a examples/paper_assets/candidates/jmavsim_prior_v1 \
  --label-a "jMAVSim prior SDF" \
  --candidate-b /tmp/sitl_reidentified_candidate \
  --label-b "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_reidentified_candidate/parameter_report
```

Parameter comparison outputs:

- markdown table:
  `/tmp/sitl_reidentified_candidate/parameter_report/parameter_summary.md`
- json table:
  `/tmp/sitl_reidentified_candidate/parameter_report/parameter_summary.json`

Prepare the re-identified SDF
-----------------------------
```bash
cd ~/px4-system-identification

python3 experimental_validation/prepare_identified_model.py \
  --candidate-dir /tmp/sitl_reidentified_candidate \
  --model-name x500_ident_matrix_reidentified
```

Re-identified 5-trajectory validation set
-----------------------------------------
Visible re-identified smoke test:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir /tmp/sitl_reidentified_candidate \
  --model-label reidentified_sitl \
  --gz-model x500_ident_matrix_reidentified \
  --display-name "Re-identified from SITL ident" \
  --trajectory-names circle \
  --visual \
  --show-console \
  --out-root /tmp/sitl_reidentified_visual
```

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir /tmp/sitl_reidentified_candidate \
  --model-label reidentified_sitl \
  --gz-model x500_ident_matrix_reidentified \
  --display-name "Re-identified from SITL ident" \
  --out-root /tmp/sitl_reidentified_suite
```

Outputs:

- dataset root:
  `/tmp/sitl_reidentified_suite/reidentified_sitl`
- tracking logs:
  `/tmp/sitl_reidentified_suite/reidentified_sitl/tracking_logs/*.csv`

Final four-layer figures and interactive review:

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --compare-root /tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 /tmp/sitl_reidentified_suite/reidentified_sitl \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_three_model_figures

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --compare-root /tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 /tmp/sitl_reidentified_suite/reidentified_sitl \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_three_model_review
```

Review outputs:

- static PNGs:
  `/tmp/sitl_three_model_figures/group_1_circle_hairpin_lemniscate.png`
- static PNGs:
  `/tmp/sitl_three_model_figures/group_2_time_optimal_minimum_snap.png`
- interactive 3D HTML:
  `/tmp/sitl_three_model_review/index.html`

Build the final three-model figures
-----------------------------------
This overlays:

- reference trajectory
- stock `x500`
- `jMAVSim`-prior SDF
- re-identified SDF

So each panel contains four curve layers in total:

1. reference
2. stock SITL result
3. prior-SDF SITL result
4. re-identified-SDF SITL result

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite/stock_sitl_placeholder \
  --compare-root /tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 /tmp/sitl_reidentified_suite/reidentified_sitl \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_three_model_figures
```

Outputs:

- `/tmp/sitl_three_model_figures/group_1_circle_hairpin_lemniscate.png`
- `/tmp/sitl_three_model_figures/group_2_time_optimal_minimum_snap.png`
- `/tmp/sitl_three_model_figures/comparison_summary.json`

Figure interpretation:

- the dashed black curve is the reference trajectory
- the blue curve is the stock `x500` SITL result
- the orange curve is the `jMAVSim`-prior SDF SITL result
- the purple curve is the re-identified SDF SITL result
- the title of each panel prints raw reference-frame `RMSE` for the three simulated curves
- the reference itself has no RMSE entry because it is the target signal
- the color bars still show along-path shape error, not the title RMSE

One-command shortcut
--------------------
If you do not want to run the whole matrix by hand, the repo now has a single
orchestrator that performs the same stages:

```bash
cd ~/px4-system-identification

python3 experimental_validation/run_sitl_three_model_validation.py \
  --out-root /tmp/sitl_three_model_validation
```

This command:

- reuses cached outputs when they already exist
- prepares the `jMAVSim`-prior model
- runs the `9/9` ident suite
- builds the re-identified candidate
- collects all three validation datasets
- writes the final PNG figures and summary JSON

Important:

- this full-matrix command is meant for dataset production
- for motion inspection in Gazebo, use the visible smoke commands first
- do not rely on the headless full matrix when the goal is to visually inspect flight behavior

Useful checks after each step
-----------------------------
- stock dataset summary:
  `/tmp/sitl_stock_suite/stock_sitl_placeholder/collection_summary.json`
- prior dataset summary:
  `/tmp/sitl_jmavsim_prior_suite/jmavsim_prior_sitl/collection_summary.json`
- ident suite manifest:
  `/tmp/sitl_ident_jmavsim_prior/jmavsim_prior_ident_suite/run_manifest.json`
- re-identified dataset summary:
  `/tmp/sitl_reidentified_suite/reidentified_sitl/collection_summary.json`
- final comparison summary:
  `/tmp/sitl_three_model_figures/comparison_summary.json`

Troubleshooting
---------------
- If a new SITL run behaves strangely, run the cleanup script again before retrying.
- If visual mode is not needed, prefer the headless collectors for the full 5-trajectory and 9-profile suites.
- If you only want to visually inspect a changed model on one trajectory, use the collector with `--trajectory-names circle --visual --show-console` and the matching `--model-label/--gz-model`.
- The `PX4 SITL Log (read-only)` window is not an interactive shell. Use the manual `make px4_sitl gz_x500` path if you need direct `pxh>` commands.
