PX4 SITL Workflow Guide
=======================

Scope
-----
This README is the repeatable `SITL` workflow for this repo.

The workflow is intentionally visual-first:

1. clean all old `PX4/Gazebo/helper` leftovers without closing `QGroundControl`
2. watch one visible stock `x500 + circle` smoke test
3. watch the full visible `5/5` stock validation suite
4. generate stock figures and pin them into `docs/sitl_validation`
5. prepare the `jMAVSim`-prior SDF, watch its visible suite, and pin those outputs too
6. run the visible `9/9` SITL identification suite on the prior model
7. build a new candidate from those ident logs
8. watch the visible `5/5` validation suite for the re-identified model
9. build the final four-layer figures and pin them into `docs/sitl_validation`

Fixed project rules
-------------------
These are intentionally stable and should not be changed between tests:

- every new top-level SITL test starts with cleanup
- the SITL flight flow is fixed:
  `arm -> takeoff -> hover -> offboard -> modules -> trajectory/ident -> POSCTL -> land`
- takeoff, hover, offboard hold, and landing use the same locked yaw policy in SITL
- `SIM_GZ_EC_MIN/MAX` is not tuned per trajectory or per ident profile
- if a changed SDF flies poorly under the fixed SITL mapping, we fix the model, not the run command
- trajectory figure titles show raw `RMSE` against the reference trajectory
- the interactive HTML review keeps full pan, zoom, and rotate controls

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

- closes visible `PX4 Gazebo Nested Display` and PX4 log windows
- stops stale `PX4 SITL`, `Gazebo`, console tail, helper, and old runner processes
- releases common serial ports used by `HITL/QGC`
- leaves `QGroundControl` open so you can watch the vehicle state there during SITL
- does not intentionally close browser windows such as `Firefox`
- if you stop a visual run with `Ctrl+C`, the runner now also triggers the same background cleanup on exit

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

- the PX4 log window is a live log viewer, not an interactive `pxh>` shell
- visual mode uses the fixed near top-down follow view
- visual runs now use the repo's marker world, so a red reference marker is always visible near takeoff
- the runner already applies the required no-RC/no-GCS SITL preflight settings
- if you stop this with `Ctrl+C`, the next visual launch should still reopen cleanly

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

Visible suite:

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

What to expect:

- Gazebo reopens sequentially for all five trajectories
- the terminal prints `[1/5]`, `[2/5]`, and so on
- while one trajectory is running, the collector may look "idle"; that is normal

Headless alternative for faster dataset production:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label stock_sitl_placeholder \
  --out-root /tmp/sitl_stock_suite
```

Stock-only figures:

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --out-dir /tmp/sitl_stock_suite_visual/stock_only_figures

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --out-dir /tmp/sitl_stock_suite_visual/stock_only_review

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section stock \
  --figures-root /tmp/sitl_stock_suite_visual/stock_only_figures \
  --review-root /tmp/sitl_stock_suite_visual/stock_only_review
```

Pinned docs outputs after the publish step:

- [Stock grouped PNG 1](docs/sitl_validation/stock/figures/group_1_circle_hairpin_lemniscate.png)
- [Stock grouped PNG 2](docs/sitl_validation/stock/figures/group_2_time_optimal_minimum_snap.png)
- <a href="docs/sitl_validation/stock/review/index.html" target="_blank" rel="noopener">Open stock interactive 3D review in a new tab</a>

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

Save the same table as markdown/json:

```bash
cd ~/px4-system-identification

python3 experimental_validation/summarize_candidate_parameters.py \
  --candidate-a examples/paper_assets/candidates/jmavsim_prior_v1 \
  --label-a "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_parameters

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section jmavsim_prior_parameters \
  --parameter-report-root /tmp/sitl_jmavsim_prior_parameters
```

Pinned docs outputs after the publish step:

- [jMAVSim prior parameter table](docs/sitl_validation/jmavsim_prior_parameters/parameters/parameter_summary.md)

jMAVSim-prior 5-trajectory validation set
-----------------------------------------
Visible smoke test:

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

Visible 5/5 suite:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir examples/paper_assets/candidates/jmavsim_prior_v1 \
  --model-label jmavsim_prior_sitl \
  --gz-model x500_ident_matrix_prior \
  --display-name "jMAVSim prior SDF" \
  --visual \
  --show-console \
  --out-root /tmp/sitl_jmavsim_prior_suite_visual
```

Headless alternative:

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

Stock vs jMAVSim-prior figures:

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --compare-root /tmp/sitl_jmavsim_prior_suite_visual/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_suite_visual/stock_vs_prior_figures

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --compare-root /tmp/sitl_jmavsim_prior_suite_visual/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --out-dir /tmp/sitl_jmavsim_prior_suite_visual/stock_vs_prior_review

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section stock_vs_prior \
  --figures-root /tmp/sitl_jmavsim_prior_suite_visual/stock_vs_prior_figures \
  --review-root /tmp/sitl_jmavsim_prior_suite_visual/stock_vs_prior_review \
  --parameter-report-root /tmp/sitl_jmavsim_prior_parameters
```

Pinned docs outputs after the publish step:

- [Stock vs prior grouped PNG 1](docs/sitl_validation/stock_vs_prior/figures/group_1_circle_hairpin_lemniscate.png)
- [Stock vs prior grouped PNG 2](docs/sitl_validation/stock_vs_prior/figures/group_2_time_optimal_minimum_snap.png)
- <a href="docs/sitl_validation/stock_vs_prior/review/index.html" target="_blank" rel="noopener">Open stock vs prior interactive 3D review in a new tab</a>
- [Pinned prior parameter table](docs/sitl_validation/stock_vs_prior/parameters/parameter_summary.md)

jMAVSim-prior 9-profile SITL identification suite
-------------------------------------------------
This runs the same fixed SITL flight flow, but switches the payload phase from
validation trajectories to the nine built-in identification profiles.

Visible suite:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/run_sitl_ident_suite.py \
  --out-root /tmp/sitl_ident_jmavsim_prior_visual \
  --label jmavsim_prior_ident_suite \
  --model-name x500_ident_matrix_prior \
  --source-model-dir ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior \
  --visual \
  --show-console
```

Headless alternative:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/run_sitl_ident_suite.py \
  --out-root /tmp/sitl_ident_jmavsim_prior \
  --label jmavsim_prior_ident_suite \
  --model-name x500_ident_matrix_prior \
  --source-model-dir ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior
```

Interactive ident review:

```bash
cd ~/px4-system-identification

python3 experimental_validation/build_hitl_review_bundle.py \
  --log-root /tmp/sitl_ident_jmavsim_prior_visual/jmavsim_prior_ident_suite \
  --out-dir /tmp/sitl_ident_jmavsim_prior_visual/review

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section prior_ident \
  --review-root /tmp/sitl_ident_jmavsim_prior_visual/review
```

Pinned docs outputs after the publish step:

- <a href="docs/sitl_validation/prior_ident/review/index.html" target="_blank" rel="noopener">Open prior ident interactive review in a new tab</a>

Build a new candidate from the SITL ident logs
----------------------------------------------
This uses the existing ident code path. The reference SDF is the prepared
`jMAVSim`-prior model that was actually flown during the ident suite.

```bash
cd ~/px4-system-identification

python3 experimental_validation/compare_with_sdf.py \
  --results-root /tmp/sitl_ident_jmavsim_prior_visual/jmavsim_prior_ident_suite \
  --out-dir /tmp/sitl_reidentified_candidate \
  --sdf-model ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior/model.sdf \
  --sdf-base-model ~/PX4-Autopilot-Identification/Tools/simulation/gz/models/x500_ident_matrix_prior_base/model.sdf
```

jMAVSim prior vs re-identified parameter table:

```bash
cd ~/px4-system-identification

python3 experimental_validation/summarize_candidate_parameters.py \
  --candidate-a examples/paper_assets/candidates/jmavsim_prior_v1 \
  --label-a "jMAVSim prior SDF" \
  --candidate-b /tmp/sitl_reidentified_candidate \
  --label-b "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_reidentified_candidate/parameter_report

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section reidentified_parameters \
  --parameter-report-root /tmp/sitl_reidentified_candidate/parameter_report
```

Pinned docs outputs after the publish step:

- [Prior vs re-identified parameter comparison](docs/sitl_validation/reidentified_parameters/parameters/parameter_summary.md)

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
Visible smoke test:

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

Visible 5/5 suite:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh

python3 experimental_validation/collect_sitl_tracking_dataset.py \
  --candidate-dir /tmp/sitl_reidentified_candidate \
  --model-label reidentified_sitl \
  --gz-model x500_ident_matrix_reidentified \
  --display-name "Re-identified from SITL ident" \
  --visual \
  --show-console \
  --out-root /tmp/sitl_reidentified_suite_visual
```

Headless alternative:

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

Final four-layer figures and interactive review
-----------------------------------------------
Each panel contains four curve layers:

1. reference
2. stock SITL result
3. prior-SDF SITL result
4. re-identified-SDF SITL result

```bash
cd ~/px4-system-identification

python3 experimental_validation/trajectory_comparison_figures.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --compare-root /tmp/sitl_jmavsim_prior_suite_visual/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 /tmp/sitl_reidentified_suite_visual/reidentified_sitl \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_three_model_figures_visual

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root /tmp/sitl_stock_suite_visual/stock_sitl_placeholder \
  --stock-label "Stock x500 SITL" \
  --compare-root /tmp/sitl_jmavsim_prior_suite_visual/jmavsim_prior_sitl \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 /tmp/sitl_reidentified_suite_visual/reidentified_sitl \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir /tmp/sitl_three_model_review_visual

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section three_model \
  --figures-root /tmp/sitl_three_model_figures_visual \
  --review-root /tmp/sitl_three_model_review_visual \
  --parameter-report-root /tmp/sitl_reidentified_candidate/parameter_report
```

Pinned docs outputs after the publish step:

- [Final grouped PNG 1](docs/sitl_validation/three_model/figures/group_1_circle_hairpin_lemniscate.png)
- [Final grouped PNG 2](docs/sitl_validation/three_model/figures/group_2_time_optimal_minimum_snap.png)
- <a href="docs/sitl_validation/three_model/review/index.html" target="_blank" rel="noopener">Open final interactive 3D review in a new tab</a>
- [Pinned prior vs re-identified parameter comparison](docs/sitl_validation/three_model/parameters/parameter_summary.md)

One-command shortcut
--------------------
If you do not want to run the whole matrix by hand, the repo still has a single
orchestrator that performs the same stages:

```bash
cd ~/px4-system-identification

python3 experimental_validation/run_sitl_three_model_validation.py \
  --out-root /tmp/sitl_three_model_validation
```

Important:

- this full-matrix command is mainly for dataset production
- it reuses cached outputs when available
- for motion inspection in Gazebo, use the visible per-stage commands above

Useful checks after each step
-----------------------------
- stock dataset summary:
  `/tmp/sitl_stock_suite_visual/stock_sitl_placeholder/collection_summary.json`
- prior dataset summary:
  `/tmp/sitl_jmavsim_prior_suite_visual/jmavsim_prior_sitl/collection_summary.json`
- ident suite manifest:
  `/tmp/sitl_ident_jmavsim_prior_visual/jmavsim_prior_ident_suite/run_manifest.json`
- re-identified dataset summary:
  `/tmp/sitl_reidentified_suite_visual/reidentified_sitl/collection_summary.json`
- final comparison summary:
  `/tmp/sitl_three_model_figures_visual/comparison_summary.json`

Troubleshooting
---------------
- If a new SITL run behaves strangely, run the cleanup script again before retrying.
- If you only want to inspect one changed model quickly, use `--trajectory-names circle --visual --show-console`.
- The `PX4 SITL Log (read-only)` window is not an interactive shell. Use `make px4_sitl gz_x500` if you need direct `pxh>` commands.
- The visible collectors reopen Gazebo sequentially. A short wait between trajectories is normal.
