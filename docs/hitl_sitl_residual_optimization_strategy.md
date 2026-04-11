# HITL-SITL Residual Optimization Strategy

## Problem Reframing

Current evidence shows that:

- shared PX4 control modules are not enough to guarantee matching HITL and SITL trajectory bodies
- exact or near-exact physical parameter transfer into SDF does not by itself produce good tube overlap
- therefore the remaining mismatch is not only a physics-identification problem

This means the practical goal should be reframed from:

- "find the exact SDF of the vehicle"

to:

- "find a physically grounded SITL model plus a small residual calibration layer that reproduces HITL trajectory tubes"

## Recommended Two-Stage Method

### Stage 1: Physical-Core Identification

Use onboard-only identification maneuvers to recover the major physically meaningful terms:

- mass
- inertia
- drag / damping family
- effective actuator lag

These parameters should come from dedicated maneuvers and should remain as fixed as possible during later calibration.

### Stage 2: Residual Calibration In SITL

After the physical core is fixed, calibrate only a small residual parameter family in SITL so that SITL reproduces HITL tubes.

This residual family should represent simulator-gap effects, for example:

- thrust mapping scale / curvature
- yaw torque scale
- rotor drag / rolling moment residual scales
- effective motor time constant scale
- body-axis damping correction
- simulator-specific slowdown / actuator transport surrogate

This is not "cheating" if:

- the residual family is low-dimensional
- the physical core stays mostly fixed
- the calibration trajectory is held separate from validation trajectories

## Overfitting Risk

Yes, if we optimize directly on a single trajectory, the model can overfit badly.

Typical failure mode:

- the model reproduces lemniscate very well
- but fails on hairpin, circle, or minimum-snap

Therefore a single-trajectory optimization should never be treated as final validation.

## Anti-Overfitting Rules

### 1. Separate Calibration And Validation

Recommended split:

- primary calibration trajectory: `lemniscate`
- optional short auxiliary residual profile:
  - collective authority / motor lag sensitive
  - bidirectional acceleration sensitive
- hold-out validation trajectories:
  - `hairpin`
  - `circle`
  - `time_optimal`
  - `minimum_snap`

If later needed:

- use `circle` as a second calibration trajectory before touching `hairpin`
- use `hairpin` as a second calibration trajectory only after first hold-out analysis

### 2. Optimize Tube Similarity, Not RMSE Alone

The objective should prioritize shape overlap and repeatability tube agreement.

Primary metrics:

- tube overlap percentage
- contact percentage
- mean center distance

Secondary metrics:

- timing alignment
- speed profile mismatch
- tilt envelope mismatch
- final settle error

### 3. Use Repeated Runs

Both HITL and SITL are not bitwise deterministic.

Therefore objective values should be computed from multiple runs:

- calibration set: `5 HITL + 5 SITL`
- validation set: `5 HITL + 5 SITL`

The optimizer should compare mean tube and spread, not a single run.

### 4. Keep Residual Family Small

Do not optimize every SDF field.

Good target:

- 6 to 12 residual parameters

Bad target:

- dozens of unconstrained SDF values

The larger the search space, the easier it is to memorize one trajectory and lose generalization.

### 5. Constrain Residual Parameters

Each optimized parameter should have:

- a nominal prior value
- a bounded search interval
- optional regularization penalty for leaving the prior too far

This keeps the calibrated model physically plausible.

### 6. Penalize Validation Collapse

Even during calibration, include light penalties that discourage pathological solutions:

- excessive tilt
- unrealistic thrust saturation
- unstable takeoff / settle behavior
- excessive mismatch in speed envelope

## Proposed Optimization Formulation

Let:

- `theta_phys` = physical-core parameters from ident maneuvers
- `theta_res` = small residual calibration vector

Freeze most of `theta_phys`, and optimize only `theta_res`.

Suggested objective:

`J(theta_res) = w1 * center_distance(lemniscate_tubes)`
`             + w2 * (1 - overlap(lemniscate_tubes))`
`             + w3 * spread_mismatch(lemniscate_tubes)`
`             + w4 * speed_profile_error`
`             + w5 * regularization(theta_res)`

Then validate on hold-out trajectories without retuning.

## Which Optimization Family Fits Best

For this problem, gradient-free optimization is the most realistic.

Good candidates:

- CMA-ES
- Bayesian optimization
- differential evolution
- Nelder-Mead for very small parameter sets

Because:

- simulator objectives are noisy
- non-smooth failures happen
- analytical gradients are not trustworthy

## Recommended Practical Path

### Phase A

Use ident maneuvers to set:

- mass
- inertia
- drag
- effective actuator lag

### Phase B

Add a primary calibration trajectory:

- `lemniscate`

Then add a short residual-sensitive maneuver if needed:

- collective authority pulse
- bidirectional acceleration pulse
- short stop-go authority probe

Current first candidate:

- `bridge_probe_xy`

Optimize only a small residual family against:

- `lemniscate` tube overlap
- and, if used, the auxiliary residual probe consistency

### Phase C

Evaluate hold-out:

- `hairpin`
- `circle`
- `time_optimal`
- `minimum_snap`

If `lemniscate` improves but all hold-outs fail, the model is overfit.

### Phase D

If needed, move from one calibration trajectory to two:

- `lemniscate + circle`

But keep `hairpin` hold-out until late, because it is a strong generalization test.

## Recommendation For The Current Project

This direction is more correct than continuing to search for a pure-SDF exact twin.

Why:

- we already found strong evidence of structural HITL-SITL mismatch
- physical-core identification still matters and should be preserved
- residual calibration gives a practical route to validation-quality agreement

So the correct interpretation is not:

- "the method failed"

It is:

- "the method discovered that pure parameter transfer is insufficient, so the problem should be upgraded to physics-informed residual calibration"

## Final Research Framing

The publishable contribution can become:

- onboard-only physical-core identification for multicopters
- followed by residual simulator calibration for HITL-to-SITL bridge validation
- evaluated using repeatability tube overlap on hold-out trajectories

This is stronger and more honest than claiming exact digital twinning from SDF parameters alone.
