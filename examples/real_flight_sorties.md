Real-flight sortie structure
===========================

Recommended sequence for outdoor identification and digital-twin validation on a calm day

Sortie 1: Mass and hover thrust
- manual takeoff to about 3 m
- keep the baseline PX4 PID path active
- switch to offboard only after hover is stable
- run `hover_thrust`
- run `mass_vertical`
- return to hover
- land

Sortie 2: Roll and pitch inertia
- manual takeoff to about 3 m
- switch to offboard
- run `roll_sweep`
- run `pitch_sweep`
- return to hover
- land

Sortie 3: Yaw inertia and motor dynamics
- manual takeoff to about 3 m
- switch to offboard
- run `yaw_sweep`
- run `motor_step`
- return to hover
- land

Sortie 4: Drag
- manual takeoff to about 3 m
- switch to offboard
- run `drag_x`
- run `drag_y`
- run `drag_z`
- return to hover
- land

Sortie 5: Validation trajectories A
- manual takeoff to about 3 m
- switch to offboard
- run validation trajectory 1: hairpin
- run validation trajectory 2: lemniscate / figure-eight
- return to hover
- land

Sortie 6: Validation trajectories B
- manual takeoff to about 3 m
- switch to offboard
- run validation trajectory 3: circle
- run validation trajectory 4: 30 s time-optimal path-parameterized mission
- run validation trajectory 5: 50 s minimum-snap mission
- return to hover
- land

If the battery budget is tighter than expected, keep the same five trajectory labels but distribute them across more batteries.

What to compare after the flights
- identify parameters from Sorties 1-4
- write them into the Gazebo candidate SDF
- replay the five validation trajectories in Gazebo
- overlay real versus simulated position traces
- report RMSE, delay, and the blended twin score
