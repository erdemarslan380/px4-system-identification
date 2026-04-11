# HITL vs SITL Infrastructure Gap Analysis

## Purpose

This note answers a narrow but critical question:

Can we reasonably expect Gazebo SITL and jMAVSim HITL to produce the same flight tubes by changing only the SDF dynamic parameters?

Short answer:

- No, not with the current stack.
- Some core control modules are shared.
- But the end-to-end infrastructure is **not** identical.
- There are several structural differences that can create divergence even if mass/inertia/drag/motor parameters are matched perfectly.

## What Is The Same

These parts are genuinely shared:

- The PX4 control modules we use for the handover and trajectory flow are the same code:
  - `custom_pos_control`
  - `trajectory_reader`
- The high-level mission idea is also aligned:
  - arm
  - takeoff / hover
  - offboard handover
  - `custom_pos_control`
  - `trajectory_reader`
  - trajectory execution

Relevant files:

- [`src/modules/custom_pos_control/custom_pos_control.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/custom_pos_control/custom_pos_control.cpp)
- [`src/modules/trajectory_reader/trajectory_reader.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/trajectory_reader/trajectory_reader.cpp)
- [`experimental_validation/run_sitl_validation.py`](/home/earsub/px4-system-identification/experimental_validation/run_sitl_validation.py)
- [`examples/run_hitl_px4_builtin_trajectory_minimal.py`](/home/earsub/px4-system-identification/examples/run_hitl_px4_builtin_trajectory_minimal.py)

## Flight-Body-Relevant Control Chain

The most important point for trajectory body shape is this:

- the **control law implementation** is mostly shared
- but the **state signals and timing that drive that law are not the same**

### Shared Control Law

These are shared in both sides:

- `custom_pos_control`
  - fixed `10 ms` schedule, i.e. `100 Hz`
  - [`custom_pos_control.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/custom_pos_control/custom_pos_control.cpp)
- `mc_pos_control`
  - same source file on both stacks
  - callback-driven from `vehicle_local_position`
  - [`MulticopterPositionControl.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mc_pos_control/MulticopterPositionControl.cpp)
- `mc_att_control`
  - same source file on both stacks
  - callback-driven from `vehicle_attitude`
  - [`mc_att_control_main.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mc_att_control/mc_att_control_main.cpp)
- `mc_rate_control`
  - same source file on both stacks
  - callback-driven from `vehicle_angular_velocity`
  - [`MulticopterRateControl.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mc_rate_control/MulticopterRateControl.cpp)

So if the question is:

> “Is the controller source code the same?”

Then the answer is mostly yes.

### Not-Shared State And Timing Inputs

The same controller code is being driven by different upstream timing/state chains.

`mc_pos_control` is triggered by `vehicle_local_position`.
`mc_att_control` is triggered by `vehicle_attitude`.
`mc_rate_control` is triggered by `vehicle_angular_velocity`.

That means the body of the trajectory depends heavily on:

- where those topics come from
- how often they update
- what latency and filtering they carry
- whether they include estimator resets/corrections

That is where SITL and HITL diverge strongly.

## What Is Not The Same

### 1. Estimator Chain Is Different

HITL explicitly disables EKF2:

- [`1001_rc_quad_x.hil`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil)
  - `param set EKF2_EN 0`

PX4 startup launches EKF2 whenever `EKF2_EN == 1`:

- [`rcS`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d/rcS)

SITL validation does **not** enforce the same HIL estimator bypass. Its base Gazebo x500 airframe is a normal POSIX/Gazebo airframe:

- [`4001_gz_x500`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500)

Implication:

- HITL is running on simulator-provided state injection.
- SITL is running in the normal Gazebo/PX4 estimator chain.
- This alone is enough to cause different delay, filtering, and state behavior even with identical SDF parameters.

### 1a. Why This Matters For Flight Body

This is not a small detail.

In HITL, `vehicle_attitude` and `vehicle_local_position` are directly published from `HIL_STATE_QUATERNION` inside MAVLink receiver:

- [`mavlink_receiver.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mavlink/mavlink_receiver.cpp)
- [`mavlink_receiver.h`](/home/earsub/PX4-Autopilot-Identification/src/modules/mavlink/mavlink_receiver.h)

Important built-in HIL publish intervals:

- attitude publish interval: `4 ms` (`250 Hz`)
- local position publish interval: `10 ms` (`100 Hz`)

So in HITL:

- the multicopter controllers are being driven by direct simulator-fed state topics
- no EKF2 estimation loop is shaping those same control inputs

In SITL:

- `vehicle_local_position` comes through the normal PX4 estimator path
- `mc_pos_control` therefore sees estimator-shaped state, not direct injected HIL state

This difference can absolutely change trajectory body shape even if the reference and SDF match.

### 2. Physics Engines Are Different

Gazebo SITL:

- Uses Gazebo + SDF + gz plugins.
- World update is configured at `250 Hz` in the Gazebo world:
  - [`Tools/simulation/gz/worlds/default.sdf`](/home/earsub/PX4-Autopilot-Identification/Tools/simulation/gz/worlds/default.sdf)

jMAVSim HITL:

- Uses the Java jMAVSim vehicle model.
- Its main loop is driven by Java `scheduleAtFixedRate(...)`:
  - [`Simulator.java`](/home/earsub/PX4-Autopilot-Identification/Tools/simulation/jmavsim/jMAVSim/src/me/drton/jmavsim/Simulator.java)
- In our HIL flow it is not running as true SITL lockstep by default:
  - [`examples/start_jmavsim_hitl.sh`](/home/earsub/px4-system-identification/examples/start_jmavsim_hitl.sh)

Implication:

- Gazebo and jMAVSim do not share the same rigid-body, rotor, and scheduler model.
- An exact SDF match is therefore **not** enough to guarantee identical closed-loop motion.

### 2a. Time Base Is Also Different

Gazebo SITL:

- syncs PX4 time to Gazebo simulation time on every clock callback
- [`GZBridge.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge/GZBridge.cpp)

jMAVSim HITL:

- uses Java scheduled execution
- optionally supports lockstep in theory
- but our HIL stack is not the same as standard SITL lockstep
- [`Simulator.java`](/home/earsub/PX4-Autopilot-Identification/Tools/simulation/jmavsim/jMAVSim/src/me/drton/jmavsim/Simulator.java)

So even the notion of “same loop time” is only partially aligned:

- controller nominal rates may be matched
- but simulator progression and transport cadence are still different

### 3. Sensor Path Is Different

Gazebo SITL:

- sends simulated IMU, baro, mag, GPS and other sensor topics into PX4 through `gz_bridge`
- PX4 then runs the normal estimator stack
- [`GZBridge.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge/GZBridge.cpp)

jMAVSim HITL:

- sends `HIL_SENSOR`, `HIL_GPS`, and `HIL_STATE_QUATERNION`
- PX4 MAVLink receiver publishes regular `vehicle_attitude` and `vehicle_local_position` directly from HIL state
- [`SimulatorMavlink.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)
- [`mavlink_receiver.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mavlink/mavlink_receiver.cpp)

Implication:

- SITL is fundamentally more “sensor -> estimator -> controller”
- HITL is fundamentally more “sim state -> MAVLink -> controller”

That is a first-order body-shape difference, not a tiny implementation detail.

### 3. Transport Path Is Different

HITL actuator path:

- PX4 hardware sends `HIL_ACTUATOR_CONTROLS` over `/dev/ttyACM0` at `200 Hz`:
  - [`rcS`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d/rcS)
  - [`mavlink_main.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/mavlink/mavlink_main.cpp)
- Then data passes through:
  - USB CDC
  - `mavlink_serial_hub.py`
  - PTY
  - jMAVSim serial input
- jMAVSim applies controls on incoming `HIL_ACTUATOR_CONTROLS` and advances time there:
  - [`MAVLinkHILSystem.java`](/home/earsub/PX4-Autopilot-Identification/Tools/simulation/jmavsim/jMAVSim/src/me/drton/jmavsim/MAVLinkHILSystem.java)

SITL actuator path:

- PX4 runs locally in `px4_sitl_default`.
- Gazebo plugins talk to PX4 through the SITL simulation bridge rather than a physical USB CDC transport:
  - [`src/modules/simulation/gz_bridge`](/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge)

Implication:

- HITL has a real serial transport bottleneck and scheduling jitter path.
- SITL does not.
- This is a structural mismatch, not an SDF mismatch.

### 4. Boot/Runtime Parameter Baseline Is Different

HITL startup forcibly applies a special baseline:

- [`examples/restart_hitl_px4_clean_gui.sh`](/home/earsub/px4-system-identification/examples/restart_hitl_px4_clean_gui.sh)
  - `SYS_HITL = 1`
  - `EKF2_EN = 0`
  - `IMU_INTEG_RATE = 250`
  - `IMU_GYRO_RATEMAX = 1000`
  - `MAV_0_RATE = 1200`
  - sensor calibration offsets reset to zero

SITL validation does not mirror that full HIL baseline. Instead it configures only the pieces needed for the Gazebo validation workflow:

- [`experimental_validation/run_sitl_validation.py`](/home/earsub/px4-system-identification/experimental_validation/run_sitl_validation.py)

Implication:

- The control modules are shared, but the full PX4 runtime environment is not identical.

### 5. Takeoff / Handover / Landing Scripts Are Not The Same

SITL validation:

- Uses a POSCTL/manual-control assisted takeoff path.
- Then performs direct hover stabilization.
- Then starts `custom_pos_control` and `trajectory_reader`.
- Then lands with a POSCTL/manual-control landing helper.

Relevant section:

- [`run_sitl_validation.py`](/home/earsub/px4-system-identification/experimental_validation/run_sitl_validation.py)

HITL validation:

- Uses clean reboot helpers.
- Uses module-managed hold wrappers and HIL-specific offboard handover.
- Uses a different direct-hover/module-handover flow.

Relevant files:

- [`run_hitl_clean_gui_trajectory.sh`](/home/earsub/px4-system-identification/examples/run_hitl_clean_gui_trajectory.sh)
- [`run_hitl_px4_module_position_hold.py`](/home/earsub/px4-system-identification/examples/run_hitl_px4_module_position_hold.py)
- [`run_hitl_px4_builtin_trajectory_minimal.py`](/home/earsub/px4-system-identification/examples/run_hitl_px4_builtin_trajectory_minimal.py)

Implication:

- The ordering is similar in spirit, but the actual entry, handover, and exit chain is not literally identical.

### 5a. Takeoff/Handover Is Not Just Cosmetic

The body of the flown trajectory depends on the initial state at trajectory start:

- position
- velocity
- tilt
- yaw
- controller warm-up state
- estimator state

SITL and HITL currently reach the trajectory start through different helper flows.
Even if both “look healthy”, they do not guarantee the same internal controller state at trajectory start.

### 6. Logging / Storage Backend Is Different

HITL:

- Full PX4 ULog backend is disabled on purpose:
  - [`1001_rc_quad_x.hil`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil)
  - `SDLOG_BACKEND = 0`
- We rely on `trajectory_reader` custom CSV logging on NuttX storage.

SITL:

- Uses the normal POSIX runtime and rootfs file paths.
- Also relies on `trajectory_reader` custom CSV logs for validation extraction.

Relevant logging code:

- [`trajectory_reader.cpp`](/home/earsub/PX4-Autopilot-Identification/src/modules/trajectory_reader/trajectory_reader.cpp)
- [`rc.logging`](/home/earsub/PX4-Autopilot-Identification/ROMFS/px4fmu_common/init.d/rc.logging)

Implication:

- Even after our buffering fixes, file I/O substrate is still different between NuttX HIL and POSIX SITL.

## Bottom-Line Verdict

We cannot truthfully say:

> “The infrastructures are the same, therefore any remaining mismatch must come only from SDF parameters.”

That statement is false for the current system.

What we **can** say:

- The high-level validation logic is aligned.
- The core trajectory/control modules are shared.
- But the end-to-end simulation infrastructure is **not** identical.
- Therefore a pure SDF-parameter identification workflow has a hard ceiling.

## Practical Consequence

If the goal is:

> “SITL should reproduce HITL flight tubes closely enough that repeatability tubes intersect meaningfully.”

then SDF identification alone is not sufficient. We need two layers:

1. **Physical-core identification**
   - mass
   - inertia
   - drag
   - effective actuator lag

2. **Bridge residual calibration**
   - simulator-specific thrust mapping residual
   - yaw/torque residual
   - actuator transport / effective lag residual
   - other bridge-only corrections

This is why exact jMAVSim-prior SDF inserted into Gazebo still failed to overlap well with HITL.

## Stronger Verdict For The User's Main Question

If the question is:

> “Can we say the two stacks are identical enough that any remaining mismatch must be SDF only?”

The answer is:

- No.

If the question is:

> “Are the flight-control source files themselves different?”

The answer is:

- Mostly no, the major controllers are shared.

If the question is:

> “Can the same control law still produce different trajectory bodies?”

The answer is:

- Yes, very easily, because the controller is being driven by different state/timing/physics chains.

## Strong Recommendation

Do not frame the remaining mismatch as:

- “we just have not found the right SDF yet”

Instead frame it as:

- “we have a shared PX4 control core, but two different simulation infrastructures”
- “digital-twin validation therefore needs both identified physics and an explicit HITL↔SITL bridge residual model”

## What This Means For The Research Program

This is still a valid and publishable direction.

A strong methodology is:

1. Build a robust onboard-only physical-core identifier.
2. Keep calibration on one symmetric trajectory, such as lemniscate.
3. Hold out hairpin and the other trajectories for validation.
4. Evaluate by tube overlap and centerline distance, not just RMSE.
5. Treat remaining mismatch as simulator-bridge residual, not just “wrong inertia/mass”.
