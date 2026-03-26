Real-flight sortie structure
===========================

Recommended sequence for outdoor identification on a calm day

Sortie 1: Mass and hover thrust
- manual takeoff to about 3 m
- switch to offboard
- enable `sysid`
- run `hover_thrust`
- run `mass_vertical`
- land

Sortie 2: Roll and pitch inertia
- manual takeoff to about 3 m
- switch to offboard
- run `roll_sweep`
- run `pitch_sweep`
- land

Sortie 3: Yaw inertia and motor dynamics
- manual takeoff to about 3 m
- switch to offboard
- run `yaw_sweep`
- run `motor_step`
- land

Sortie 4: Drag
- manual takeoff to about 3 m
- switch to offboard
- run `drag_x`
- run `drag_y`
- run `drag_z`
- land
