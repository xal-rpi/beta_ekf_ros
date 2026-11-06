std_msgs/Header header

bool initialized
bool vehicle_input_stale

# State and diagonal covariance order:
# [pos_e, pos_n, psi, vel_x, vel_y, yaw_rate,
#  rear_wheelspeed_ms, deltaFx_hat, deltaFy_hat, mu_scale]
float32[10] state
float32[10] variance

float32 beta_rad

# Debug only: ||F[3..8, mu_scale]||_2 of the discrete process Jacobian,
# evaluated at the current EKF state with mu_scale replaced by debug_mu_scale_ref
# and a fixed nominal dt = 1 / loop_rate_hz. NaN when debug_j_mu_enable is false.
float32 debug_j_mu
float32 debug_mu_scale_ref

# Debug only: model tire forces at the current EKF state (estimated mu_scale) and
# inputs, from xal_tire_forces. NaN when debug_tire_util_enable is false.
# debug_tire_forces: [Fx_f, Fy_f, Fz_f, Fx_r, Fy_r, Fz_r], tire/wheel frame [N]
# debug_gamma: [front, rear] utilization of each axle's friction limit
#   (mu_y includes mu_scale; mu_x is frozen at the checkpoint value):
#   front (friction circle): gamma_f = sqrt(Fx_f^2 + Fy_f^2) / (mu_y_f Fz_f)
#   rear  (Fiala ellipse):   gamma_r = sqrt((Fx_r/(mu_x_r Fz_r))^2 + (Fy_r/(mu_y_r Fz_r))^2)
#   also NaN below debug_tire_util_min_speed, where slip is ill-defined.
float32[6] debug_tire_forces
float32[2] debug_gamma

# Friction information from this cycle's accepted fusions:
# I_mu = sum (H xi)^T R^-1 (H xi), with xi = dx/d(mu_scale).
# c_mu = sigmoid(a (q_mu - 0.5)), q_mu = clip((I_mu_filt - I_mu_low) / (I_mu_high - I_mu_low), 0, 1),
# scales the mu_scale Kalman-gain row when tuning.mu_obs_enable is true.
# mu_scale and P_mu are state[9] and variance[9].
float32 i_mu
float32 i_mu_filt
float32 c_mu

# Model-validity domain for the mu_scale update (tuning.mu_domain_enable).
# c_mu above already includes w_domain. w_domain = 0 while either axle's tire
# utilization gamma_domain (evaluated at mu_scale - 2*sqrt(P_mu)) exceeds
# tuning.mu_domain_gamma_max, the front wheels lock (kappa_front <
# -tuning.mu_domain_kappa_lock), or vx <= 0.5 m/s, and for tuning.mu_domain_hold_s
# afterwards. w_domain is 1 and the other two NaN when the domain check is off.
float32 w_domain
float32 gamma_domain
float32 kappa_front

bool gps_velocity_accepted
bool gyro_accepted
bool wheel_speed_accepted
bool accel_accepted
float32 fuse_freq
