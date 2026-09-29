std_msgs/Header header

bool initialized
bool vehicle_input_stale

# State and diagonal covariance order:
# [pos_e, pos_n, psi, vel_x, vel_y, yaw_rate,
#  rear_wheelspeed_ms, deltaFx_hat, deltaFy_hat]
float32[9] state
float32[9] variance

float32 beta_rad

bool gps_velocity_accepted
bool gyro_accepted
bool wheel_speed_accepted
bool accel_accepted
float32 fuse_freq
