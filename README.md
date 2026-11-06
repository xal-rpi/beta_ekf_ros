# beta_ekf_ros

ROS 2 wrapper around the existing PX4-Drive `BetaEKF`. The estimator and
CasADi-generated model are compiled directly from
`src/modules/simple_sub_pub`; this package does not copy or modify them.

## Build

From the PX4-Drive repository root:

```sh
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select beta_ekf_ros \
  --cmake-args \
    -DBETA_EKF_CORE_DIR="$PWD/src/modules/simple_sub_pub"
source install/setup.bash
```

The CMake option can be omitted when the package remains under
`PX4-Drive/ros2/beta_ekf_ros`.

## Run

```sh
ros2 run beta_ekf_ros beta_ekf_node
```

All topics are private and can be remapped. The default interfaces are:

- Input `/EGO/sensors/control_state/control_state` (`bng_msgs/BngVehicleStateMsg`), representing the live BeamNG vehicle plant state.
- Output `~/state` (`beta_ekf_ros/BetaEkfState`).
- Output `~/odometry` (`nav_msgs/Odometry`).
- Output `~/beta` (`std_msgs/Float32`).
- Service `~/reset` (`std_srvs/Trigger`).

The node initializes from the first valid vehicle-state sample. Its timer calls
the EKF predict step at `loop_rate_hz`, which defaults to 200 Hz. Until a valid
vehicle-state message arrives, the process-model inputs remain at their zeroed
initial values.

## Loop and measurement rate

`measurement_fuse_ratio` uses the same accumulator approach as PX4's
`SimpleSubPub::shouldFuseSensor()`:

- `1.0`: fuse every newly received measurement set.
- `0.5`: fuse approximately every second measurement set.
- `0.1`: fuse approximately every tenth measurement set.
- `0.0`: disable measurement fusion while prediction continues.

With fresh measurements available on every 200 Hz loop, ratios `1.0`, `0.5`,
and `0.1` correspond to approximately 200, 100, and 20 Hz. If a sensor publishes
more slowly than the loop, the effective fusion rate is its publish rate
multiplied by the ratio. A skipped sample is discarded rather than fused later.

The ratio can be changed while the node is running:

```sh
ros2 param set /beta_ekf measurement_fuse_ratio 0.5
```

Changing `loop_rate_hz` or a topic-name parameter requires restarting the node.

GPS velocity, gyro, and wheel-speed fusion can also be enabled independently
with `fuse_gps_velocity`, `fuse_gyro`, and `fuse_wheel_speed`.

## Changing topics or message types

The following startup parameters select topic names:

- `vehicle_state_topic`
- `state_topic`
- `odometry_output_topic`
- `beta_topic`

For example:

```sh
ros2 run beta_ekf_ros beta_ekf_node --ros-args \
  -p loop_rate_hz:=200.0 \
  -p measurement_fuse_ratio:=0.5 \
  -p vehicle_state_topic:=/EGO/sensors/control_state/control_state
```

ROS topic types are fixed at compile time. If the eventual inputs use different
message types, edit only the clearly marked subscription-adapter section and
its callbacks in `beta_ekf_node.cpp`; the fixed-rate estimator loop can remain
unchanged.

## Frames and units

ROS messages use ENU world and FLU body axes. The wrapper converts those data
to the EKF API's NED world and FRD body convention. The live BeamNG state is
provided as body-frame velocity and orientation quaternion. Inputs use SI units:
torque in N m, steering in radians, wheel speed in m/s, and normalized brake in
the inclusive range `[0, 1]`.

Every `BetaEKFTuning` member is available under the `tuning.*` parameter
namespace. The `tuning.*` parameters are re-read on every estimator loop, so
they can be changed live with `ros2 param set`. Topic names and `loop_rate_hz`
still require a restart.

## Tuning

### Process noise (`tuning.q_*`)

Continuous noise intensities, multiplied by `dt` internally. Larger values make
the state follow the measurements faster and trust the model less.

| Parameter | Default | State |
|---|---|---|
| `tuning.q_pos_e`, `tuning.q_pos_n` | 0.01 | position |
| `tuning.q_psi` | 0.01 | heading |
| `tuning.q_vx`, `tuning.q_vy` | 0.5 | body velocity |
| `tuning.q_yaw_rate` | 0.1 | yaw rate |
| `tuning.q_rear_ws` | 1.0 | rear wheel speed |
| `tuning.q_delta_fx`, `tuning.q_delta_fy` | 1.0 | filtered specific forces (`fcog_x`/`fcog_y`) |

The `mu_scale` process noise is fixed in code at `1e-4 * dt`.

### Measurement noise (`tuning.r_*`)

Measurement variances. Smaller values mean the sensor is trusted more.

| Parameter | Default | Measurement |
|---|---|---|
| `tuning.r_vx`, `tuning.r_vy` | 0.8 (m/s)² | GPS body velocity |
| `tuning.r_yaw_rate` | 0.01 (rad/s)² | gyro z |
| `tuning.r_rear_ws` | 0.25 (m/s)² | rear wheel speed |
| `tuning.r_accel_x`, `tuning.r_accel_y` | 1.0 (m/s²)² | accelerometer |
| `tuning.r_beta` | (10°)² | sideslip (when fused) |

`tuning.gate_gamma_max` (default 1600) is the Mahalanobis innovation gate, and
`tuning.v_min` (default 0.3 m/s) is the minimum speed for beta fusion.
Accelerometer fusion is always on; GPS velocity, gyro and wheel speed are
switched with the `fuse_*` parameters.

> **Note:** `I_mu` (below) scales with `1/R`. After changing any `r_*` value or
> `measurement_fuse_ratio`, re-check `I_mu_low` / `I_mu_high`.

### Observability-aware `mu_scale` update

`mu_scale` is the tire-road friction scale. It multiplies the lateral friction
(`fiala_mu`) of the exported model only; the longitudinal friction
(`fiala_mu_x`) is frozen at its checkpoint value. (Scaling both was tried, but
hard braking then biased `mu_scale` to about 1.4 on asphalt, because the model
under-predicts the front brake force when the front wheels lock.) Friction is
only observable when the tires work hard, so the `mu_scale` correction is
weighted by how much friction information the measurements carry.
Once per loop:

1. The friction sensitivity `ξ = ∂x/∂mu_scale` is propagated with the model
   Jacobian and corrected by `(I − K H)` after each accepted fusion.
2. The information `I_mu = Σ (Hξ)ᵀ R⁻¹ (Hξ)` is summed over the accepted
   fusions, then low-pass filtered: `Ī = λ·Ī + (1 − λ)·I_mu`.
3. The confidence is
   `q_mu = clip((Ī − I_mu_low) / (I_mu_high − I_mu_low), 0, 1)` and
   `c_mu = 1 / (1 + exp(−a·(q_mu − 0.5)))`.
4. When enabled, only the `mu_scale` row of the Kalman gain is multiplied by
   `c_mu`; the existing Joseph covariance update keeps state and covariance
   consistent. P_mu is then floored at
   `P_min = p_high − c_mu·(p_high − p_low)`.

| Parameter | Default | Meaning |
|---|---|---|
| `tuning.mu_obs_enable` | false | Turns on gain scaling and the P_mu floor. `i_mu`, `i_mu_filt` and `c_mu` are always computed and published. |
| `tuning.mu_obs_lambda` | 0.99 | Low-pass factor; time constant ≈ `dt / (1 − λ)` ≈ 0.5 s at 200 Hz |
| `tuning.I_mu_low` | 0.0 | `i_mu_filt` level of low-information driving (c_mu ≈ 0) |
| `tuning.I_mu_high` | 1.0 | `i_mu_filt` level of informative maneuvers (c_mu ≈ 1). Must be > `I_mu_low`. |
| `tuning.mu_confidence_sigmoid_a` | 10.0 | Sharpness of the transition. Must be > 0. With a = 10, c_mu goes from 0.007 to 0.993; with a = 4, from 0.12 to 0.88. |
| `tuning.mu_obs_p_high` | 0.25 | P_mu floor at c_mu = 0 |
| `tuning.mu_obs_p_low` | 1e-3 | P_mu floor at c_mu = 1 |

Invalid values are clamped (`I_mu_high` to `I_mu_low + 1e-9`, `a` to `1e-3`),
and the node prints a throttled warning.

Recommended procedure:

1. Keep `tuning.mu_obs_enable` false. Record `/casadi_ekf/state` while driving
   straight, mild cornering, aggressive cornering, and hard acceleration and
   braking.
2. Plot `i_mu_filt` on a log scale. Set `I_mu_low` to the straight-driving
   level and `I_mu_high` to the level held during clearly informative maneuvers.
3. Read the settled `variance[9]` (P_mu) from the same run; call it P_base.
4. Set `tuning.mu_obs_p_high ≈ P_base` and `tuning.mu_obs_p_low ≈ 0.1·P_base`.
   **Do not leave `p_high` at 0.25.** That keeps P_mu large, which makes
   `mu_scale` jumpy and lets it absorb model mismatch (observed: `mu_scale`
   ≈ 2.0 on asphalt). Setting both to 0 disables the floor.
5. Enable `tuning.mu_obs_enable`, reset the filter, and compare `mu_scale` with
   the disabled run.

Because `mu_scale` scales lateral friction only, `I_mu` comes mainly from
cornering. Pure acceleration or rear-wheel slip carries no friction
information; hard braking does only when the front axle reaches its friction
circle.

```sh
ros2 param set /beta_ekf tuning.I_mu_low 0.0
ros2 param set /beta_ekf tuning.I_mu_high 0.8
ros2 param set /beta_ekf tuning.mu_confidence_sigmoid_a 10.0
ros2 param set /beta_ekf tuning.mu_obs_enable true
ros2 service call /casadi_ekf/reset std_srvs/srv/Trigger
```

### Model-validity domain (`tuning.mu_domain_*`)

`I_mu` measures how strongly friction affects the model, not whether the model
is right. Friction matters most when tires slide (spins, lock-ups, hard braking
in a turn), and that is exactly where the vehicle model is wrong. Without a
validity check these maneuvers dominate the `mu_scale` update.

The domain check sets a weight `w_domain` that multiplies `c_mu` (the gain
only; the P_mu floor keeps using the information-based confidence, so frozen
periods do not inflate P_mu). `w_domain = 0` while any of these holds, and for
`mu_domain_hold_s` afterwards:

- either axle's tire utilization `gamma_domain` > `mu_domain_gamma_max`.
  γ is evaluated at a cautious `mu_scale − 2·√P_mu`, so an over-estimated
  `mu_scale` cannot hide saturation;
- the front wheels lock: measured front slip `kappa_front` <
  `−mu_domain_kappa_lock` (the model has no front wheel-speed state);
- the vehicle is not moving forward (`vx ≤ 0.5 m/s`).

γ is normalized by friction (γ = 0.8 means 80 % of the available grip on any
surface), so these limits are not specific to asphalt.

| Parameter | Default | Meaning |
|---|---|---|
| `tuning.mu_domain_enable` | false | Turns the domain check on |
| `tuning.mu_domain_gamma_max` | 0.85 | Upper tire-utilization limit |
| `tuning.mu_domain_kappa_lock` | 0.3 | Front-lock slip threshold |
| `tuning.mu_domain_hold_s` | 0.5 s | Hold time after leaving the domain |

The defaults come from validation run 089 (techground asphalt, `mu_scale` held
at 1.0). Up to γ ≈ 0.8 the model fits well (normalized fit error 0.3, best-fit
`mu_scale` 1.10 [1.05–1.15]). At saturation (γ ≈ 1) the fit error is about
6× larger and the best-fit `mu_scale` is wrong (0.75). After saturation the fit
recovers within 0.3 s. Below γ ≈ 0.6 friction has no measurable effect on the
model, so `I_mu` is low there anyway.

```sh
ros2 param set /beta_ekf tuning.mu_domain_enable true
```

The scripted validation sequence that produced these numbers is in
`python_src/beamng_open_loop_control.ipynb` (last cell). The full analysis,
with figures, is in [docs/mu_domain_validation.md](docs/mu_domain_validation.md).

## Debug features

Both diagnostics are off by default, because each adds one extra CasADi model
evaluation per loop. They can be toggled while the node runs and do not change
the EKF state, covariance or gain.

### Process-model friction sensitivity (`debug_j_mu`)

`debug_j_mu = ‖F[3..8, mu_scale]‖₂` is the `mu_scale` column of the discrete
process Jacobian over the vehicle rows (`vx`, `vy`, `yaw_rate`, `rear_ws`,
`fcog_x`, `fcog_y`), excluding the `mu_scale → mu_scale` self-term. It is
evaluated at the current EKF state, but with `mu_scale` replaced by
`debug_mu_scale_ref` and with a fixed `dt = 1 / loop_rate_hz`, so it reflects
the maneuver rather than the estimate. It is near 0 when cruising, rises with
lateral tire saturation (and front-axle saturation under braking), and is ≈ 0
below 0.5 m/s.

| Parameter | Default |
|---|---|
| `debug_j_mu_enable` | false |
| `debug_mu_scale_ref` | 1.0 (live) |
| `debug_mu_sensitivity_topic` | `casadi_ekf/debug_mu_sensitivity` (restart) |

Outputs: `debug_j_mu` and `debug_mu_scale_ref` in `BetaEkfState` (NaN when
disabled), and a `std_msgs/Float32` on the debug topic, published only while
enabled.

### Tire forces and utilization (`debug_tire_util`)

Model tire forces at the current EKF state, using the estimated `mu_scale`:

- `debug_tire_forces = [Fx_f, Fy_f, Fz_f, Fx_r, Fy_r, Fz_r]`, tire frame [N].
- `debug_gamma = [front, rear]`, the utilization of each axle's friction limit:
  - front (friction circle): `√(Fx² + Fy²) / (mu_y·Fz)`
  - rear (Fiala ellipse): `√((Fx / (mu_x·Fz))² + (Fy / (mu_y·Fz))²)`

γ ≈ 1 means the axle is at its friction limit. γ is a model value, so an error
in `mu_scale` shows up directly in it (too high a `mu_scale` gives too low a
γ). `debug_gamma` is NaN below `debug_tire_util_min_speed`, where slip is
ill-defined.

| Parameter | Default |
|---|---|
| `debug_tire_util_enable` | false |
| `debug_tire_util_min_speed` | 2.0 m/s (live) |

```sh
ros2 param set /beta_ekf debug_j_mu_enable true
ros2 topic echo /casadi_ekf/debug_mu_sensitivity

ros2 param set /beta_ekf debug_tire_util_enable true
ros2 topic echo /casadi_ekf/state --field debug_gamma
```

## Friction fields in `BetaEkfState`

| Field | Meaning |
|---|---|
| `state[9]`, `variance[9]` | `mu_scale` and its variance P_mu |
| `i_mu`, `i_mu_filt` | Raw and filtered friction information |
| `c_mu` | Confidence used to scale the `mu_scale` gain (when enabled); includes `w_domain` |
| `w_domain`, `gamma_domain`, `kappa_front` | Model-validity weight and the cautious tire utilization / front slip it is based on |
| `debug_j_mu`, `debug_mu_scale_ref` | Debug Jacobian sensitivity (NaN when off) |
| `debug_tire_forces`, `debug_gamma` | Debug tire forces and utilization (NaN when off) |
