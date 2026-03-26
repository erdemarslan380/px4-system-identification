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

Sortie 5: Validation trajectories
- manual takeoff to about 3 m
- switch to offboard
- run validation trajectory 1: smooth lemniscate / figure-eight
- run validation trajectory 2: climbing spiral
- run validation trajectory 3: aggressive box / cornered path
- return to hover
- land

If the battery budget is tight, split Sortie 5 into two shorter validation sorties.

What to compare after the flights
- identify parameters from Sorties 1-4
- write them into the Gazebo candidate SDF
- replay the three validation trajectories in Gazebo
- overlay real versus simulated position traces
- report RMSE, delay, and the blended twin score
