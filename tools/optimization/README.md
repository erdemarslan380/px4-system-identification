# Optimization Toolkit

This folder contains the browser-based controller optimization workflow.

The three user views are:
- **Planner**
- **Live Monitor**
- **Results Review**

The simulator backend is now **Gazebo**.

## What This Toolkit Does

It lets you:
- prepare a study with several tasks
- run controller optimization task by task
- monitor progress in real time
- inspect finished results without disturbing the live run
- replay finished solutions in an isolated Gazebo runtime

## URLs

Start the local web server:

```bash
cd ~/px4-custom
python3 Tools/optimization/serve_dashboard.py \
  --port 8090 \
  --root ~/px4-custom/Tools/optimization
```

Then use:
- Planner: `http://127.0.0.1:8090/planner`
- Live Monitor: `http://127.0.0.1:8090/dashboard`
- Results Review: `http://127.0.0.1:8090/review`

## Supported Controllers

- `pid`
- `dfbc`
- `indi`
- `mpc`
- `cmpc`
- `sysid`

## Supported Optimizers

- `bayes`
- `random`
- `anneal`

## Planner

Planner is for building a study.

You can:
- open an existing study
- create a new study
- add several tasks
- delete tasks
- reorder tasks
- choose controller, optimizer, and trajectory
- preview the trajectory in a rotatable 3D view
- edit parameter limits directly

The Planner writes study files to:
- `Tools/optimization/generated_plans/`

## Live Monitor

Live Monitor is only for the active study.

It shows:
- current task
- queue progress
- worker activity
- objective history
- live trajectory trace

Use it to answer:
- Is the study still running?
- Which task is active?
- How many evaluations are finished?
- Which workers are busy?

## Results Review

Results Review is for completed tasks.

It shows:
- task tables
- start vs best metrics
- percentage improvements
- finished traces
- isolated replay controls

Important idea:
- Live Monitor watches the active run.
- Results Review studies completed runs.
- Replay is isolated and should not be used as part of the optimizer itself.

## Mission Logic Used by the Evaluator

Each task uses the same evaluation structure:
1. PX4-controlled takeoff
2. short stabilized hover
3. target controller engagement
4. OFFBOARD dwell
5. trajectory execution
6. landing

The objective window is the **OFFBOARD segment**.

This keeps the optimization focused on controller behavior during the tracked mission, not on the takeoff or landing transient.

## Run a Study from the Command Line

```bash
cd ~/px4-custom
python3 Tools/optimization/run_simulation_plan.py \
  --plan Tools/optimization/example_plan.yaml \
  --serve-dashboard
```

## Example Study

The reference example is:
- `Tools/optimization/example_plan.yaml`

It contains:
- multiple controllers
- multiple trajectories
- optimization settings
- Gazebo simulator defaults

## Gazebo Notes

The toolkit launches Gazebo in standalone mode internally.

For PX4 + Gazebo integration, the code uses the same official PX4 Gazebo path described in:
- https://docs.px4.io/main/en/sim_gazebo_gz/

The current toolkit defaults are:
- simulator: `gz`
- vehicle: `x500`
- world: `default`

These values can be changed in a study file.

## Identification Missions

The planner and evaluator now support two task types:
- `trajectory`
- `identification`

Use `identification` with controller `sysid` when you want PX4 to generate built-in excitation motions and export a rich CSV for model estimation.

The current identification motions are:
- `hover_thrust`
- `mass_vertical`
- `roll_sweep`
- `pitch_sweep`
- `yaw_sweep`
- `drag_x`
- `drag_y`
- `drag_z`
- `motor_step`

Reference study files:
- `Tools/optimization/generated_plans/x500_identification_suite.yaml`
- `Tools/optimization/generated_plans/x500_identification_comprehensive.yaml`
- `Tools/optimization/generated_plans/all_controllers_quick_validation.yaml`
- `Tools/optimization/generated_plans/overnight_bayes_viable.yaml`
- `Tools/optimization/generated_plans/overnight_bayes_8g2l.yaml` (manual long study)

You can also attach a QGroundControl full-parameter dump with:
- `base_param_file`

Recommended location:
- `experimental_validation/qgc/current_vehicle.params`

## Long unattended runs

Use the watchdog when you want the study to resume unfinished tasks automatically after an unexpected runner stop:

```bash
cd ~/px4-custom
python3 Tools/optimization/plan_watchdog.py \
  --plan Tools/optimization/generated_plans/overnight_bayes_viable.yaml \
  --clean-start \
  --serve-dashboard
```

The watchdog keeps the active plan in `resume` mode for `pending`, `running`, and `failed` tasks until the study finishes or the resume budget is exhausted.

## Validation Commands

### UI contract tests

```bash
python3 -m unittest Tools.optimization.tests.test_ui_contracts
```

### Watchdog contract tests

```bash
python3 -m unittest Tools.optimization.tests.test_watchdog
```

### Planner browser smoke

```bash
/tmp/codex-selenium/bin/python Tools/optimization/tests/ui_browser_smoke.py --port 8090 --headless
```

### Planner -> Live -> Review smoke

```bash
python3 Tools/optimization/tests/flow_http_smoke.py --port 8090
```

### Gazebo end-to-end smoke

```bash
python3 Tools/optimization/tests/gz_e2e_smoke.py --port 8094
```

This last test validates:
- planner save/start path
- Gazebo-backed optimization
- finished trace access from review
- isolated replay in Gazebo

### Gazebo identification smoke

```bash
python3 Tools/optimization/tests/gz_identification_smoke.py --port 8096
```

This validates:
- `sysid` task execution in Gazebo
- identification CSV export
- experimental-validation CLI output generation
- review trace access
- isolated replay for the identification task

## Key Files

- `serve_dashboard.py`
- `run_simulation_plan.py`
- `orchestrate_parallel.py`
- `parallel_pool.py`
- `px4_eval.py`
- `visual_replay.py`
- `simulator_backend.py`

## Output Folders

- generated plans: `Tools/optimization/generated_plans/`
- study runs: `Tools/optimization/plan_runs/`
- shared runtime files: `Tools/optimization/results/`

## Related Folder

For real-flight-to-Gazebo model identification, use:
- [experimental_validation/README.md](/home/earsub/px4-custom/experimental_validation/README.md)
