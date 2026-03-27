Real-flight sortie structure
===========================

Use this sequence on a calm day. The baseline controller for these sorties is the PX4 default controller path. The overlay is only used to generate the identification maneuvers and logs.

Before every sortie
-------------------
1. power the vehicle and verify a healthy RC link,
2. start `custom_pos_control` and `trajectory_reader`,
3. keep the vehicle in manual mode,
4. arm and take off manually,
5. stabilize at about `3 m`,
6. switch to `OFFBOARD` only after the hover is healthy,
7. wait for the vehicle to settle before sending the next identification command.

During every identification maneuver
------------------------------------
- send only one `trajectory_reader set_ident_profile ...` command,
- wait for these PX4 console messages before moving on:
  - `Identification maneuver started: ...`
  - `Identification maneuver completed: ...`
  - `Identification log completed: ...`
  - `Tracking log completed: ...`
- after completion, the vehicle holds the final reference,
- recover to a clean hover before the next command.

Maneuver table
--------------
| Profile | Purpose | Approx. duration | After completion |
| --- | --- | ---: | --- |
| `hover_thrust` | hover-thrust tracking around a constant hover point | `26 s` | hold final reference |
| `mass_vertical` | multi-frequency vertical motion for mass and thrust scale | `36 s` | hold final reference |
| `roll_sweep` | roll-axis inertia and coupling | `28 s` | hold final reference |
| `pitch_sweep` | pitch-axis inertia and coupling | `28 s` | hold final reference |
| `yaw_sweep` | yaw inertia and yaw moment balance | `24 s` | hold final reference |
| `drag_x` | X-axis drag | `30 s` | hold final reference |
| `drag_y` | Y-axis drag | `30 s` | hold final reference |
| `drag_z` | Z-axis drag | `30 s` | hold final reference |
| `motor_step` | motor time constants | `24 s` | hold final reference |

Recommended identification sorties
----------------------------------
Sortie 1: mass and hover thrust
- manual takeoff to about `3 m`
- stabilize hover
- switch to OFFBOARD
- run `hover_thrust`
- hover recovery
- run `mass_vertical`
- hover recovery
- land

Sortie 2: roll and pitch inertia
- manual takeoff to about `3 m`
- stabilize hover
- switch to OFFBOARD
- run `roll_sweep`
- hover recovery
- run `pitch_sweep`
- hover recovery
- land

Sortie 3: yaw and motor dynamics
- manual takeoff to about `3 m`
- stabilize hover
- switch to OFFBOARD
- run `yaw_sweep`
- hover recovery
- run `motor_step`
- hover recovery
- land

Sortie 4: drag
- manual takeoff to about `3 m`
- stabilize hover
- switch to OFFBOARD
- run `drag_x`
- hover recovery
- run `drag_y`
- hover recovery
- run `drag_z`
- hover recovery
- land

Validation sorties after identification
---------------------------------------
The five validation trajectories are:
- `hairpin`
- `lemniscate`
- `circle`
- `time_optimal_30s`
- `minimum_snap_50s`

All five are defined so the logged mission segment:
- starts from the same XY point,
- starts from the same altitude (`3 m`, i.e. `z = -3` in NED),
- returns to the same logged start pose.

For each validation sortie:
1. take off manually to about `3 m`,
2. stabilize hover,
3. switch to OFFBOARD,
4. move to the common trajectory start pose,
5. begin logging only after the vehicle reaches that start pose,
6. execute one or two validation trajectories,
7. return to hover,
8. land.

If the battery margin is tighter than expected, keep the same trajectory names but distribute them across more batteries.

Log locations
-------------
On the vehicle storage, the CSV logs should appear under:
- `/fs/microsd/identification_logs/`
- `/fs/microsd/tracking_logs/`

After each sortie, copy them into a session folder such as:
- `~/px4-system-identification/experimental_validation/outputs/real_flights/2026-03-27_session_01/raw_logs/`

What to compare after the flights
---------------------------------
1. identify parameters from Sorties 1-4,
2. write them into the Gazebo candidate SDF,
3. simulate the five validation trajectories in Gazebo,
4. overlay the real-flight traces and the digital-twin traces,
5. report RMSE, delay, and the blended twin score.
