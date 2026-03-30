# Architecture

## Repository Purpose
This repository is a standalone PX4 system-identification toolkit. It separates identification work from the older optimization/planner stack and keeps only the pieces needed to:
- run identification maneuvers,
- collect PX4 and Gazebo logs,
- estimate SDF parameters,
- validate a digital twin,
- generate publication-ready artifacts.

## Top-Level Layout
- `README.md`
  Main setup and usage guide.
- `system_identification.txt`
  Long-form method description for papers, reports, and prompt-based writing workflows.
- `overlay/`
  PX4/Gazebo source overlay copied into a PX4 workspace before build.
- `experimental_validation/`
  Python estimation, comparison, scoring, and figure-generation pipeline.
- `examples/`
  Example YAML plans, sortie definitions, and generated paper assets.
- `sync_into_px4_workspace.sh`
  Copies the overlay into an external PX4 workspace.

## Overlay Structure
- `overlay/src/modules/custom_pos_control/`
  PX4-side control entry point. In this repo it is limited to baseline PX4-default behavior plus system-identification support.
- `overlay/src/modules/trajectory_reader/`
  Built-in identification motion generator, trajectory playback logic, and the campaign state machine for `identification_only`, `trajectory_only`, and `full_stack`.
- `overlay/src/modules/simulation/gz_plugins/system_identification_logger/`
  Gazebo plugin that logs simulator truth needed for SITL upper-bound validation.

## Experimental Validation Structure
- `experimental_validation/identification.py`
  Loads logs, aligns truth rows, estimates parameters, and builds identification reports.
- `experimental_validation/estimators.py`
  Lower-level estimators for mass, inertia, drag, motor coefficients, and motor dynamics.
- `experimental_validation/compare_with_sdf.py`
  Compares identified values with the SDF reference and writes JSON summaries.
- `experimental_validation/twin_metrics.py`
  Computes the blended digital-twin score and family-level scores.
- `experimental_validation/paper_artifacts.py`
  Produces overlay plots, sensitivity plots, and summary files for documentation and papers.
- `experimental_validation/reference_models.py`
  Built-in x500 reference and currently frozen example candidate models.
- `experimental_validation/tests/`
  Regression tests for estimators, comparisons, scoring, and figure generation.

## Data Flow
1. Identification motion runs inside PX4.
2. `trajectory_reader` can execute one item at a time or a built-in campaign while suppressing return-to-anchor legs from the CSV logs.
3. PX4 writes identification logs and tracking logs.
4. In SITL, Gazebo truth logger writes synchronized truth logs.
5. `identification.py` merges and normalizes those logs.
6. Estimators recover model parameters.
7. `compare_with_sdf.py` compares recovered parameters against the known SDF reference.
8. `paper_artifacts.py` converts the current candidate into researcher-facing figures and tables.

## Validation Modes
- `px4_only`
  Uses only PX4-side identification logs.
- `telemetry_augmented`
  Uses PX4 logs plus telemetry-like fields that are also feasible to record in real flight.
- `truth_assisted`
  Uses Gazebo truth fields. This is the near-perfect SITL upper-bound validation mode.

## Design Intent
- Keep the identification method reproducible without requiring the old optimization repository.
- Let researchers run the workflow manually from documented shell commands.
- Keep outputs simple: CSV, JSON, PNG, and small YAML plans.
