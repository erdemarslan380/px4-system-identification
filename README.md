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

README rendering notes
----------------------
Two kinds of links are kept in this document:

- local repo links such as `docs/sitl_validation/.../index.html`
- GitHub-rendered HTML links served through `rawcdn.githack.com`

The local links are best when you inspect the repo on the same machine.
The rendered links are best when you inspect the same README from GitHub and
want the HTML review to open directly instead of showing the file contents.

Important limitation:

- GitHub does not execute repo HTML files directly from normal file links, so a plain `index.html` repo link may still show source or download behavior
- for a guaranteed rendered local view, start the local docs server once and use the dedicated local review app
- the dedicated local review app reads fixed CSV files from `docs/sitl_validation/_generated/sources`
- avoid opening the heavy generated review HTML directly if Firefox shows a blank `3D Path View`

Recommended local review web app:

```bash
cd ~/px4-system-identification
python3 experimental_validation/serve_sitl_docs.py \
  --open-path docs/sitl_validation/local_review/index.html
```

This serves the repo at `http://127.0.0.1:8765/` and opens:

`http://127.0.0.1:8765/docs/sitl_validation/local_review/index.html`

This is the clearest review screen. It has:

- all five trajectories: `circle`, `hairpin`, `lemniscate`, `time_optimal_30s`, `minimum_snap_50s`
- all four layers: `reference`, `stock`, `jMAVSim prior SDF`, `re-identified from SITL ident`
- local CSV reads from fixed repo paths
- 3D zoom/pan/rotate through Plotly
- a reliable SVG inspector, a 2D top-down view, layer checkboxes, track target selection, and progress sliders near both the top and `Selected point` section

How refresh works:

- the app reads CSV files directly from `docs/sitl_validation/_generated/sources`
- if you rerun one simulation and overwrite the matching CSV there, refresh the browser and the app will show the new data
- for example, re-identified `circle` is read from `docs/sitl_validation/_generated/sources/re_identified_from_sitl_ident/tracking_logs/circle.csv`
- if a new run writes to `/tmp/...`, it will not appear until that CSV is copied or published into the matching `_generated/sources/.../tracking_logs/<trajectory>.csv` path
- use `Ctrl+F5` or add a URL suffix such as `?v=3` if Firefox keeps showing cached data

Fallback direct open for the heavy generated HTML:

```bash
cd ~/px4-system-identification
xdg-open docs/sitl_validation/three_model/review/index.html
```

Permanent generated review inputs:

- the review source CSVs are pinned under `docs/sitl_validation/_generated/sources`
- the intermediate self-contained HTML builds are pinned under `docs/sitl_validation/_generated/reviews`
- the final files you open are still published under `docs/sitl_validation/<section>/review/index.html`
- these paths are intentionally not under `/tmp`, so they survive reboot unless you delete them from the repo

Regenerate the permanent trajectory review HTMLs from the pinned CSVs:

```bash
cd ~/px4-system-identification

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root docs/sitl_validation/_generated/sources/stock \
  --stock-label "Stock x500 SITL" \
  --out-dir docs/sitl_validation/_generated/reviews/stock

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root docs/sitl_validation/_generated/sources/stock \
  --stock-label "Stock x500 SITL" \
  --compare-root docs/sitl_validation/_generated/sources/jmavsim_prior_sdf \
  --compare-label "jMAVSim prior SDF" \
  --out-dir docs/sitl_validation/_generated/reviews/stock_vs_prior

python3 experimental_validation/build_sitl_trajectory_review_bundle.py \
  --stock-root docs/sitl_validation/_generated/sources/stock \
  --stock-label "Stock x500 SITL" \
  --compare-root docs/sitl_validation/_generated/sources/jmavsim_prior_sdf \
  --compare-label "jMAVSim prior SDF" \
  --compare-root-2 docs/sitl_validation/_generated/sources/re_identified_from_sitl_ident \
  --compare-label-2 "Re-identified from SITL ident" \
  --out-dir docs/sitl_validation/_generated/reviews/three_model
```

Publish those permanent review builds into the normal README links:

```bash
cd ~/px4-system-identification

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section stock \
  --review-root docs/sitl_validation/_generated/reviews/stock

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section stock_vs_prior \
  --review-root docs/sitl_validation/_generated/reviews/stock_vs_prior

python3 experimental_validation/publish_sitl_docs_assets.py \
  --section three_model \
  --review-root docs/sitl_validation/_generated/reviews/three_model
```

If Firefox shows a blank `3D Path View`, use the top `Visible Static Review`
section first. That section is inline SVG and does not need JavaScript, CSV
files, internet, or the local docs server. It should show all five trajectories
directly in the same `index.html`.

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
- <a href="docs/sitl_validation/stock/review/index.html" target="_blank" rel="noopener">Open local stock interactive review</a>
- <a href="http://127.0.0.1:8765/docs/sitl_validation/stock/review/index.html" target="_blank" rel="noopener">Open rendered stock review from local docs server</a>
- <a href="https://rawcdn.githack.com/erdemarslan380/px4-system-identification/main/docs/sitl_validation/stock/review/index.html" target="_blank" rel="noopener">Open rendered stock interactive review from GitHub</a>

Review notes:

- the review still opens the interactive inspector on `circle` by default
- the top `Visible Static Review` section shows all `5` trajectories immediately as inline SVG
- use the `Trajectory Switcher` pills for the interactive `3D Path View`
- each trajectory keeps its own embedded raw CSV download link; no external CSV files are required for the HTML to render

Embedded stock figures:

<a href="docs/sitl_validation/stock/figures/group_1_circle_hairpin_lemniscate.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/stock/figures/group_1_circle_hairpin_lemniscate.png" alt="Stock grouped figure 1" style="max-width:100%; border-radius:14px;" />
</a>

<a href="docs/sitl_validation/stock/figures/group_2_time_optimal_minimum_snap.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/stock/figures/group_2_time_optimal_minimum_snap.png" alt="Stock grouped figure 2" style="max-width:100%; border-radius:14px;" />
</a>

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

Legacy old-ident candidate snapshot
-----------------------------------
This is the older `HITL`-driven candidate that was tested before the current
`jMAVSim`-prior and `SITL re-identification` flow. It is kept in the README so
the parameter story grows in layers instead of hiding the older attempt.

Source candidate:

- `experimental_validation/outputs/x500_candidate_from_current_hil_ident/identified_parameters.json`

Important note:

- this older candidate is **not** used in the final Gazebo validation figures
- the reason is physical invalidity in its inertia terms:
  `Izz > Ixx + Iyy`
- because of that, it is shown here as a comparison table, not as a trusted
  flight model

`jMAVSim prior` vs `old ident` parameter table:

| Parameter | Unit | jMAVSim prior | Old ident |
|---|---:|---:|---:|
| Mass | kg | 0.8 | 1.0 |
| Inertia X | kg*m^2 | 0.005 | 0.011913 |
| Inertia Y | kg*m^2 | 0.005 | 3.170203e-08 |
| Inertia Z | kg*m^2 | 0.009 | 0.163773 |
| Drag X | - | 0.01 | 0 |
| Drag Y | - | 0.01 | 0 |
| Drag Z | - | 0.01 | 0 |
| Time constant up | s | 0.005 | 0 |
| Time constant down | s | 0.005 | 0 |
| Max rotor velocity | rad/s | 1000 | 0 |
| Motor constant | N/(rad/s)^2 | 4.000000e-06 | 0 |
| Moment constant | - | 0.0125 | 0 |
| Rotor drag coefficient | - | 8.064280e-05 | 0 |
| Rolling moment coefficient | - | 1.000000e-06 | 0 |
| Rotor velocity slowdown | - | 10 | 0 |

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
- <a href="docs/sitl_validation/stock_vs_prior/review/index.html" target="_blank" rel="noopener">Open local stock vs prior interactive review</a>
- <a href="http://127.0.0.1:8765/docs/sitl_validation/stock_vs_prior/review/index.html" target="_blank" rel="noopener">Open rendered stock vs prior review from local docs server</a>
- <a href="https://rawcdn.githack.com/erdemarslan380/px4-system-identification/main/docs/sitl_validation/stock_vs_prior/review/index.html" target="_blank" rel="noopener">Open rendered stock vs prior interactive review from GitHub</a>
- [Pinned prior parameter table](docs/sitl_validation/stock_vs_prior/parameters/parameter_summary.md)

Embedded stock vs prior figures:

<a href="docs/sitl_validation/stock_vs_prior/figures/group_1_circle_hairpin_lemniscate.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/stock_vs_prior/figures/group_1_circle_hairpin_lemniscate.png" alt="Stock vs prior grouped figure 1" style="max-width:100%; border-radius:14px;" />
</a>

<a href="docs/sitl_validation/stock_vs_prior/figures/group_2_time_optimal_minimum_snap.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/stock_vs_prior/figures/group_2_time_optimal_minimum_snap.png" alt="Stock vs prior grouped figure 2" style="max-width:100%; border-radius:14px;" />
</a>

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

- <a href="docs/sitl_validation/prior_ident/review/index.html" target="_blank" rel="noopener">Open local prior ident interactive review</a>
- <a href="http://127.0.0.1:8765/docs/sitl_validation/prior_ident/review/index.html" target="_blank" rel="noopener">Open rendered prior ident review from local docs server</a>
- <a href="https://rawcdn.githack.com/erdemarslan380/px4-system-identification/main/docs/sitl_validation/prior_ident/review/index.html" target="_blank" rel="noopener">Open rendered prior ident interactive review from GitHub</a>

Review notes:

- the bundle includes all `9` identification profiles
- each profile contributes both a `tracking` trace and an `identification` trace
- the interactive review therefore shows `18` selectable runs in total
- run labels now use profile names such as `mass_vertical [tracking]` and `mass_vertical [identification]`

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

`jMAVSim prior` vs `old ident` vs `new SITL ident` parameter table:

| Parameter | Unit | jMAVSim prior | Old ident | New SITL ident |
|---|---:|---:|---:|---:|
| Mass | kg | 0.8 | 1.0 | 0.8 |
| Inertia X | kg*m^2 | 0.005 | 0.011913 | 0.005 |
| Inertia Y | kg*m^2 | 0.005 | 3.170203e-08 | 0.005 |
| Inertia Z | kg*m^2 | 0.009 | 0.163773 | 0.009 |
| Drag X | - | 0.01 | 0 | -0.044726 |
| Drag Y | - | 0.01 | 0 | -0.053264 |
| Drag Z | - | 0.01 | 0 | -0.005291 |
| Time constant up | s | 0.005 | 0 | 0.0125 |
| Time constant down | s | 0.005 | 0 | 0.025 |
| Max rotor velocity | rad/s | 1000 | 0 | 1000 |
| Motor constant | N/(rad/s)^2 | 4.000000e-06 | 0 | 8.548580e-06 |
| Moment constant | - | 0.0125 | 0 | 0.016 |
| Rotor drag coefficient | - | 8.064280e-05 | 0 | 8.064280e-05 |
| Rolling moment coefficient | - | 1.000000e-06 | 0 | 1.000000e-06 |
| Rotor velocity slowdown | - | 10 | 0 | 10 |

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
- <a href="http://127.0.0.1:8765/docs/sitl_validation/local_review/index.html" target="_blank" rel="noopener">Open recommended local webserver review app</a>
- <a href="docs/sitl_validation/local_review/index.html" target="_blank" rel="noopener">Open local review app file path</a>
- <a href="docs/sitl_validation/three_model/review/index.html" target="_blank" rel="noopener">Open heavy generated final review fallback</a>
- <a href="http://127.0.0.1:8765/docs/sitl_validation/three_model/review/index.html" target="_blank" rel="noopener">Open heavy generated final review from local docs server</a>
- <a href="https://rawcdn.githack.com/erdemarslan380/px4-system-identification/main/docs/sitl_validation/three_model/review/index.html" target="_blank" rel="noopener">Open rendered final interactive review from GitHub</a>
- [Pinned prior vs re-identified parameter comparison](docs/sitl_validation/three_model/parameters/parameter_summary.md)

Review notes:

- use the recommended local webserver review app first
- the app contains all `5` trajectories and the four comparison layers
- layer visibility and track target selection are handled from the top control panel
- the heavy generated final review is kept as a fallback/archive, but the webserver app is easier to debug

Embedded final figures:

<a href="docs/sitl_validation/three_model/figures/group_1_circle_hairpin_lemniscate.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/three_model/figures/group_1_circle_hairpin_lemniscate.png" alt="Final grouped figure 1" style="max-width:100%; border-radius:14px;" />
</a>

<a href="docs/sitl_validation/three_model/figures/group_2_time_optimal_minimum_snap.png" target="_blank" rel="noopener">
  <img src="docs/sitl_validation/three_model/figures/group_2_time_optimal_minimum_snap.png" alt="Final grouped figure 2" style="max-width:100%; border-radius:14px;" />
</a>

Final 4-layer RMSE summary
--------------------------
This is the final trajectory-level scoreboard using the four visible layers in
the grouped figures:

| Trajectory | Stock RMSE [m] | jMAVSim prior RMSE [m] | Re-identified RMSE [m] |
|---|---:|---:|---:|
| Circle | 0.6616 | 0.2770 | 1.4062 |
| Hairpin | 0.6449 | 0.3503 | 0.5630 |
| Lemniscate | 0.6467 | 0.5066 | 0.5878 |
| Time optimal 30 s | 0.3897 | 0.2833 | 0.3048 |
| Minimum snap 50 s | 0.5138 | 0.5065 | 0.5136 |

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
