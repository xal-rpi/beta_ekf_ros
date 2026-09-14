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
namespace. Parameters are loaded when the node starts.
