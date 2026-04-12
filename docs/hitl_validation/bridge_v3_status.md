# HITL-SITL Bridge V3 Status

Date: 2026-04-12

This note records the latest bridge-calibration result after switching the selection metric
from single-run RMSE to repeated-tube overlap on the calibration trajectory.

## Method

- Physical-core candidate source:
  - `ident + bridge_probe_xy + lemniscate`
- Residual calibration sweep:
  - `bridge_calibration_fix4_lemniscate`
- Tube-based candidate selection:
  - repeated `lemniscate x5` HITL vs repeated `lemniscate x5` SITL
- Hold-out validation:
  - repeated `hairpin x5` HITL vs repeated `hairpin x5` SITL

## Selected Bridge Candidate

- Candidate:
  - `x500_scaled_165mm__thrust_up`
- Candidate dir:
  - `/home/earsub/px4-system-identification/sitl_runs/bridge_calibration_fix4_lemniscate/stage2/candidates/x500_scaled_165mm__thrust_up`

Residual interpretation:
- thrust/motor family adjustments are more useful than pure yaw damping or pure drag scaling
- however, the remaining HITL-to-SITL mismatch is still large

## Lemniscate Calibration Result

Review bundle:
- `/home/earsub/px4-system-identification/docs/hitl_validation/hitl_sitl_lemniscate_bridge_two_layer`

Metrics:
- mean overlap: `0.023%`
- contact: `0.286%`
- mean center distance: `1.259233 m`

## Hairpin Hold-out Result

Review bundle:
- `/home/earsub/px4-system-identification/docs/hitl_validation/hitl_sitl_hairpin_bridge_two_layer`

Metrics:
- mean overlap: `0.018%`
- contact: `1.429%`
- mean center distance: `0.751287 m`

## Current Reading

- The bridge candidate chosen by repeated lemniscate overlap does not generalize well enough to hairpin.
- Even on the calibration trajectory, tube overlap remains extremely low.
- This strongly suggests that the current residual family is still missing a dominant simulator-gap mechanism.

Most likely missing families:
- nonlinear thrust mapping residual
- yaw/torque residual beyond a single scalar scale
- actuator/transport timing residual that is not representable with simple SDF motor parameters
- state-chain/runtime differences between HITL and SITL that SDF-only tuning cannot close

## V4 In Progress

A new residual family is now being tested:

- probe-seeded directional motor residuals
- probe-seeded directional moment residuals

These are seeded from `bridge_probe_xy` indirect observables and pushed into the Gazebo motor plugins as
axis-dependent `motorConstant` / `momentConstant` balance terms.

Relevant code:
- [materialize_bridge_candidate_variant.py](/home/earsub/px4-system-identification/experimental_validation/materialize_bridge_candidate_variant.py)
- [prepare_identified_model.py](/home/earsub/px4-system-identification/experimental_validation/prepare_identified_model.py)
- [calibrate_bridge_candidate.py](/home/earsub/px4-system-identification/experimental_validation/calibrate_bridge_candidate.py)

Active selection root:
- `/home/earsub/px4-system-identification/sitl_runs/bridge_probe_balance_selection_fix5_lemniscate`

Goal:
- first re-rank candidates on `lemniscate x5` tube overlap
- then carry the best one to `hairpin x5` hold-out

## Live UI

- Dashboard:
  - `http://127.0.0.1:8765/docs/hitl_validation/hitl_sitl_bridge_dashboard/index.html`
- Lemniscate panel:
  - `http://127.0.0.1:8765/docs/hitl_validation/hitl_sitl_lemniscate_bridge_two_layer/index.html`
- Hairpin panel:
  - `http://127.0.0.1:8765/docs/hitl_validation/hitl_sitl_hairpin_bridge_two_layer/index.html`

## V5 Truth-State Bridge

- A larger hidden gap is now treated explicitly as a runtime/state-chain gap, not only an SDF parameter gap.
- Gazebo SITL bridge-calibration runs can now enable:
  - `PX4_GZ_TRUTH_STATE_BRIDGE=1`
- In that mode, Gazebo groundtruth is republished onto:
  - `vehicle_attitude` at `250 Hz`
  - `vehicle_local_position` at `100 Hz`
- This is meant to move SITL state semantics closer to the HIL path, where PX4 is effectively driven by truth-like state updates rather than a full EKF2 estimator chain.

Files:
- [GZBridge.hpp](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge/GZBridge.hpp)
- [GZBridge.cpp](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge/GZBridge.cpp)
- [run_sitl_validation.py](/home/earsub/px4-system-identification/experimental_validation/run_sitl_validation.py)
- [select_bridge_candidate_by_overlap.py](/home/earsub/px4-system-identification/experimental_validation/select_bridge_candidate_by_overlap.py)
- [calibrate_bridge_candidate.py](/home/earsub/px4-system-identification/experimental_validation/calibrate_bridge_candidate.py)

First smoke result with old candidate `x500_scaled_165mm__thrust_up`:
- `mean_center_distance_m = 0.991519`
- previous estimator-based SITL best on lemniscate was `1.259233`
- so centerline distance improved materially
- but overlap stayed `0.0%`
- and the run became vertically unstable late in the maneuver

Current next step:
- re-run candidate selection on a short residual shortlist under this truth-state bridge mode
- judge improvement by overlap/contact first, center distance second
