# multi_trajectory_setpoint.msg
uint64 timestamp                # time since boot
uint8 MAX_HORIZON = 20          # fixed array bound for small preview horizons

float32[20] position_x
float32[20] position_y
float32[20] position_z
float32[20] velocity_x
float32[20] velocity_y
float32[20] velocity_z
float32[20] accel_x
float32[20] accel_y
float32[20] accel_z
float32[20] yaw
float32[20] yawspeed

uint8 horizon_length            # actual used length (<= MAX_HORIZON)
bool valid
