# HITL-SITL Bridge V2 Status

Current best residual family after the April 11 bridge refinement pass:

- geometry preset: `x500_scaled_165mm`
- `time_constant_up_scale = 0.82`
- `time_constant_down_scale = 0.82`
- `motor_constant_scale = 1.12`
- `moment_constant_scale = 1.20`
- `rotor_drag_scale = 0.70`
- `rolling_moment_scale = 0.85`
- `slowdown_scale = 0.85`

Interpretation:

- the bridge mismatch was not reduced by extra body damping alone
- the strongest improvement came from a more aggressive motor-authority family
- this means the current dominant residual is closer to actuator/thrust/yaw behavior than to pure translational drag

Latest two-layer dashboard URLs:

- `/docs/hitl_validation/hitl_sitl_bridge_dashboard/index.html`
- `/docs/hitl_validation/hitl_sitl_lemniscate_bridge_two_layer/index.html`
- `/docs/hitl_validation/hitl_sitl_hairpin_bridge_two_layer/index.html`

Current bridge metrics:

- lemniscate HITL vs calibrated SITL:
  - mean overlap: `0.0 %`
  - contact: `0.143 %`
  - mean center distance: `1.114173 m`
- hairpin HITL vs calibrated SITL:
  - mean overlap: `0.0 %`
  - contact: `0.143 %`
  - mean center distance: `0.630398 m`

Comparison against the previous residual family:

- lemniscate mean center distance improved from `1.26638 m` to `1.114173 m`
- hairpin mean center distance improved from `0.7136 m` to `0.630398 m`

Conclusion:

- the new residual family is better
- the bridge is still not close enough to call it a digital twin
- the next likely gains should come from richer thrust/yaw residuals or other onboard-log-observable anchors, not from adding more isotropic damping
