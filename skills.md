# Skills

This repository is focused on one job: extracting Gazebo/PX4 vehicle model parameters from identification flights and turning them into a reusable digital-twin candidate.

## What This Project Does Well
- Adds a PX4-side system-identification mode through the overlay in `overlay/`.
- Generates repeatable identification maneuvers such as `hover_thrust`, `mass_vertical`, `roll_sweep`, `pitch_sweep`, `yaw_sweep`, `drag_x`, `drag_y`, `drag_z`, and `motor_step`.
- Logs the quantities needed for model recovery from PX4 and, in SITL, from Gazebo truth.
- Estimates inertial, motor, and drag parameters from those logs.
- Compares identified parameters with a Gazebo SDF reference.
- Produces paper-ready figures, validation summaries, and stress-test plots.

## Intended Users
- Control researchers
- Robotics experimenters
- PX4 users who want to build a system-identification workflow without carrying the larger optimization project

## Core Workflow Skills
1. Apply the overlay into a PX4 workspace with `sync_into_px4_workspace.sh`.
2. Build PX4 SITL or a firmware target with the overlay enabled.
3. Run identification maneuvers in Gazebo or on a real vehicle.
4. Export or copy the resulting identification CSV logs.
5. Estimate a digital-twin candidate with `experimental_validation/cli.py`.
6. Validate that candidate with `compare_with_sdf.py` and `paper_artifacts.py`.

## Assumptions
- Baseline control for identification is the PX4 default position controller.
- Real-flight validation will later reuse the same maneuver families, but without Gazebo truth fields.
- The strongest current validation mode is `truth_assisted` in SITL; this is the near-perfect upper bound.

## Current Technical Focus
- Keep truth-assisted SITL recovery near-perfect.
- Improve telemetry-only / real-flight transfer quality.
- Reduce remaining roll-axis (`Ixx`) bias in practical closed-loop identification.
