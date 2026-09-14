#include "beta_ekf_ros/frame_conversions.hpp"

#include "beta_ekf_casadi.hpp"
#include "beta_ekf_ros/msg/beta_ekf_state.hpp"
#include "bng_msgs/msg/bng_vehicle_state_msg.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace beta_ekf_ros
{

namespace
{

constexpr double kMaximumEkfStepSeconds = 0.05;
constexpr double kUnsupportedVariance = 1e6;

bool finite(double value)
{
	return std::isfinite(value);
}

}  // namespace

class BetaEkfNode : public rclcpp::Node
{
public:
	BetaEkfNode()
	: Node("beta_ekf")
	{
		loadParameters();
		createRosInterfaces();

		const auto loop_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
		estimator_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(loop_period),
			std::bind(&BetaEkfNode::runEstimator, this));

		RCLCPP_INFO(
			get_logger(),
			"Beta EKF loop %.1f Hz, measurement fuse ratio %.3f",
			loop_rate_hz_, measurement_fuse_ratio_);
	}

private:
	// -------------------------------------------------------------------------
	// Internal data used by the EKF.
	//
	// ROS callbacks below are only adapters: they translate whatever ROS
	// messages are subscribed to into these small structs. If the input message
	// types change later, keep runEstimator() unchanged and edit only the
	// subscriptions and adapter callbacks.
	// -------------------------------------------------------------------------
	struct ProcessInputs
	{
		float rear_wheel_torque_nm{0.0F};
		float roadwheel_angle_rad{0.0F};
		float brake{0.0F};
		bool received{false};
	};

	struct GpsVelocityMeasurement
	{
		float pos_e{0.0F};
		float pos_n{0.0F};
		float yaw{0.0F};
		float body_vx{0.0F};
		float body_vy{0.0F};
		float yaw_rate{0.0F};
		float velocity_ned[3]{};
		float q_nb[4]{};
		bool valid{false};
		bool updated{false};
	};

	struct GyroMeasurement
	{
		float gyro_frd[3]{};
		bool valid{false};
		bool updated{false};
	};

	struct WheelSpeedMeasurement
	{
		float rear_wheel_speed_mps{0.0F};
		bool valid{false};
		bool updated{false};
	};

	void loadParameters()
	{
		loop_rate_hz_ = declare_parameter("loop_rate_hz", 200.0);
		measurement_fuse_ratio_ = declare_parameter("measurement_fuse_ratio", 1.0);
		reset_gap_s_ = declare_parameter("reset_gap_s", 0.5);

		loop_rate_hz_ = std::max(loop_rate_hz_, 1.0);
		measurement_fuse_ratio_ = std::clamp(measurement_fuse_ratio_, 0.0, 1.0);
		reset_gap_s_ = std::max(reset_gap_s_, kMaximumEkfStepSeconds);

		fuse_gps_velocity_ = declare_parameter("fuse_gps_velocity", true);
		fuse_gyro_ = declare_parameter("fuse_gyro", true);
		fuse_wheel_speed_ = declare_parameter("fuse_wheel_speed", true);

		odom_frame_ = declare_parameter("odom_frame", "odom");
		base_frame_ = declare_parameter("base_frame", "base_link");

		// Topic names are startup parameters. They may be relative, private, or
		// absolute ROS topic names, and can also be changed with ROS remapping.
		vehicle_state_topic_ = declare_parameter("vehicle_state_topic", "/EGO/sensors/control_state/control_state");
		state_topic_ = declare_parameter("state_topic", "casadi_ekf/state");
		odometry_output_topic_ = declare_parameter("odometry_output_topic", "casadi_ekf/odometry");
		beta_topic_ = declare_parameter("beta_topic", "casadi_ekf/beta");

		BetaEKFTuning tuning{};
		tuning.q_pos_e = tuningParameter("tuning.q_pos_e", tuning.q_pos_e);
		tuning.q_pos_n = tuningParameter("tuning.q_pos_n", tuning.q_pos_n);
		tuning.q_psi = tuningParameter("tuning.q_psi", tuning.q_psi);
		tuning.q_vx = tuningParameter("tuning.q_vx", tuning.q_vx);
		tuning.q_vy = tuningParameter("tuning.q_vy", tuning.q_vy);
		tuning.q_yaw_rate = tuningParameter("tuning.q_yaw_rate", tuning.q_yaw_rate);
		tuning.q_rear_ws = tuningParameter("tuning.q_rear_ws", tuning.q_rear_ws);
		tuning.q_delta_fx = tuningParameter("tuning.q_delta_fx", tuning.q_delta_fx);
		tuning.q_delta_fy = tuningParameter("tuning.q_delta_fy", tuning.q_delta_fy);
		tuning.R_vx = tuningParameter("tuning.r_vx", tuning.R_vx);
		tuning.R_vy = tuningParameter("tuning.r_vy", tuning.R_vy);
		tuning.R_beta = tuningParameter("tuning.r_beta", tuning.R_beta);
		tuning.R_yaw_rate = tuningParameter("tuning.r_yaw_rate", tuning.R_yaw_rate);
		tuning.R_rear_ws = tuningParameter("tuning.r_rear_ws", tuning.R_rear_ws);
		tuning.v_min = tuningParameter("tuning.v_min", tuning.v_min);
		tuning.gate_gamma_max = tuningParameter("tuning.gate_gamma_max", tuning.gate_gamma_max);
		ekf_.setTuning(tuning);
	}

	float tuningParameter(const std::string &name, float default_value)
	{
		return static_cast<float>(declare_parameter(name, static_cast<double>(default_value)));
	}

	void createRosInterfaces()
	{
		const auto sensor_qos = rclcpp::SensorDataQoS();

		state_publisher_ = create_publisher<msg::BetaEkfState>(state_topic_, sensor_qos);
		odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_output_topic_, sensor_qos);
		beta_publisher_ = create_publisher<std_msgs::msg::Float32>(beta_topic_, sensor_qos);

		// ---------------------------------------------------------------------
		// ROS SUBSCRIPTION ADAPTERS
		//
		// Subscribe to the live vehicle state message and adapt it for the
		// existing EKF measurement fusion logic.
		// ---------------------------------------------------------------------
		vehicle_state_subscription_ = create_subscription<bng_msgs::msg::BngVehicleStateMsg>(
			vehicle_state_topic_, sensor_qos,
			std::bind(&BetaEkfNode::vehicleStateCallback, this, std::placeholders::_1));

		reset_service_ = create_service<std_srvs::srv::Trigger>(
			"casadi_ekf/reset",
			std::bind(
				&BetaEkfNode::resetCallback, this,
				std::placeholders::_1, std::placeholders::_2));
	}

	// -------------------------------------------------------------------------
	// ROS MESSAGE ADAPTER CALLBACKS
	// -------------------------------------------------------------------------
	void vehicleStateCallback(const bng_msgs::msg::BngVehicleStateMsg::SharedPtr message)
	{
		if (!finite(message->position.x) || !finite(message->position.y) ||
		    !finite(message->orientation.w) || !finite(message->orientation.x) ||
		    !finite(message->orientation.y) || !finite(message->orientation.z) ||
		    !finite(message->linear_velocity.x) || !finite(message->linear_velocity.y) ||
		    !finite(message->angular_velocity.z) ||
		    !finite(message->delta_l) || !finite(message->delta_r) ||
		    !finite(message->brake)) {
			return;
		}

		frames::Quaternion ros_quaternion{
			message->orientation.w,
			message->orientation.x,
			message->orientation.y,
			message->orientation.z
		};

		if (!frames::normalize(ros_quaternion)) {
			return;
		}

		const frames::Vector3 velocity_ned = frames::bodyFluToWorldNed(
			ros_quaternion,
			{message->linear_velocity.x, message->linear_velocity.y, message->linear_velocity.z});
		const frames::Quaternion q_nb = frames::rosFluEnuToCoreFrdNed(ros_quaternion);

		gps_measurement_.pos_e = static_cast<float>(message->position.x);
		gps_measurement_.pos_n = static_cast<float>(message->position.y);
		gps_measurement_.yaw = static_cast<float>(frames::yawFromRosQuaternion(ros_quaternion));
		gps_measurement_.body_vx = static_cast<float>(message->linear_velocity.x);
		gps_measurement_.body_vy = static_cast<float>(message->linear_velocity.y);
		gps_measurement_.yaw_rate = static_cast<float>(message->angular_velocity.z);

		for (int index = 0; index < 3; ++index) {
			gps_measurement_.velocity_ned[index] = static_cast<float>(velocity_ned[index]);
		}

		for (int index = 0; index < 4; ++index) {
			gps_measurement_.q_nb[index] = static_cast<float>(q_nb[index]);
		}

		gps_measurement_.valid = true;
		gps_measurement_.updated = true;

		const frames::Vector3 gyro_frd = frames::fluToFrd({
			message->angular_velocity.x,
			message->angular_velocity.y,
			message->angular_velocity.z
		});

		for (int index = 0; index < 3; ++index) {
			gyro_measurement_.gyro_frd[index] = static_cast<float>(gyro_frd[index]);
		}

		gyro_measurement_.valid = true;
		gyro_measurement_.updated = true;

		float rear_speed = std::numeric_limits<float>::quiet_NaN();
		const bool rr_ok = finite(message->w_rr);
		const bool rl_ok = finite(message->w_rl);

		if (rr_ok && rl_ok) {
			rear_speed = static_cast<float>(0.5 * (message->w_rr + message->w_rl));
		} else if (rr_ok) {
			rear_speed = static_cast<float>(message->w_rr);
		} else if (rl_ok) {
			rear_speed = static_cast<float>(message->w_rl);
		}

		if (std::isfinite(rear_speed)) {
			wheel_speed_measurement_.rear_wheel_speed_mps = rear_speed;
			wheel_speed_measurement_.valid = true;
			wheel_speed_measurement_.updated = true;
		}

		process_inputs_.roadwheel_angle_rad = static_cast<float>(0.5 * (message->delta_l + message->delta_r));
		process_inputs_.brake = static_cast<float>(message->brake);
		process_inputs_.received = true;

		if (finite(message->rear_wheel_torque_est)) {
			process_inputs_.rear_wheel_torque_nm = static_cast<float>(message->rear_wheel_torque_est);
		}
	}

	// Same accumulator logic used by SimpleSubPub::shouldFuseSensor().
	bool shouldFuseMeasurements(double ratio)
	{
		ratio = std::clamp(ratio, 0.0, 1.0);

		if (ratio <= 0.0) {
			measurement_fuse_accumulator_ = 0.0;
			return false;
		}

		if (ratio >= 1.0) {
			measurement_fuse_accumulator_ = 0.0;
			return true;
		}

		measurement_fuse_accumulator_ += ratio;

		if (measurement_fuse_accumulator_ >= 1.0) {
			measurement_fuse_accumulator_ -= 1.0;
			return true;
		}

		return false;
	}

	void initializeFromLatestOdometry()
	{
		float initial_state[BetaEKF::NX] {
			gps_measurement_.pos_e,
			gps_measurement_.pos_n,
			gps_measurement_.yaw,
			gps_measurement_.body_vx,
			gps_measurement_.body_vy,
			gps_measurement_.yaw_rate,
			std::max(gps_measurement_.body_vx, 0.1F),
			0.0F,
			0.0F
		};
		float initial_variance[BetaEKF::NX] {
			25.0F, 25.0F, 1.0F, 0.10F, 0.10F, 0.25F, 1.0F, 4.0F, 4.0F
		};

		ekf_.init(initial_state, initial_variance);
		estimator_initialized_ = true;
		reset_requested_ = false;
		last_loop_time_ = std::chrono::steady_clock::now();
		gps_measurement_.updated = false;

		RCLCPP_INFO(get_logger(), "Beta EKF initialized from the latest odometry sample");
	}

	// -------------------------------------------------------------------------
	// FIXED-RATE ESTIMATOR LOOP
	//
	// This is intentionally laid out like SimpleSubPub::Run(): compute dt,
	// predict once every loop, then optionally fuse newly received measurements.
	// -------------------------------------------------------------------------
	void runEstimator()
	{
		const auto loop_time = std::chrono::steady_clock::now();
		// Read this inexpensive parameter every loop so the fusion rate can be
		// changed at runtime with `ros2 param set`.
		measurement_fuse_ratio_ = std::clamp(
			get_parameter("measurement_fuse_ratio").as_double(), 0.0, 1.0);

		if ((!estimator_initialized_ || reset_requested_) && gps_measurement_.valid) {
			initializeFromLatestOdometry();
			publishEstimate();
			return;
		}

		if (!estimator_initialized_) {
			return;
		}

		const double dt = std::chrono::duration<double>(loop_time - last_loop_time_).count();
		last_loop_time_ = loop_time;

		if (!finite(dt) || dt <= 0.0) {
			return;
		}

		if (dt > reset_gap_s_) {
			estimator_initialized_ = false;
			reset_requested_ = true;
			RCLCPP_WARN(get_logger(), "Estimator loop gap %.3f s; reinitializing from odometry", dt);
			return;
		}

		// =====================================================================
		// 1. PREDICT -- called every timer tick (default: 200 Hz)
		// =====================================================================
		const int prediction_steps = std::max(
			1, static_cast<int>(std::ceil(dt / kMaximumEkfStepSeconds)));
		const float prediction_dt = static_cast<float>(dt / prediction_steps);

		for (int step = 0; step < prediction_steps; ++step) {
			ekf_.predict(
				process_inputs_.rear_wheel_torque_nm,
				process_inputs_.roadwheel_angle_rad,
				process_inputs_.brake,
				prediction_dt);
		}

		// =====================================================================
		// 2. FUSE MEASUREMENTS -- rate controlled by measurement_fuse_ratio
		// =====================================================================
		const bool measurement_updated =
			gps_measurement_.updated ||
			gyro_measurement_.updated ||
			wheel_speed_measurement_.updated;

		if (measurement_updated && shouldFuseMeasurements(measurement_fuse_ratio_)) {
			if (fuse_gps_velocity_ && gps_measurement_.updated && gps_measurement_.valid) {
				last_gps_accepted_ = ekf_.fuseGpsVelNED(
					gps_measurement_.velocity_ned[0],
					gps_measurement_.velocity_ned[1],
					gps_measurement_.velocity_ned[2],
					gps_measurement_.q_nb);
			}

			if (fuse_gyro_ && gyro_measurement_.updated && gyro_measurement_.valid) {
				last_gyro_accepted_ = ekf_.fuseGyro(gyro_measurement_.gyro_frd);
			}

			if (fuse_wheel_speed_ && wheel_speed_measurement_.updated && wheel_speed_measurement_.valid) {
				last_wheel_speed_accepted_ = ekf_.fuseWheelSpeed(
					wheel_speed_measurement_.rear_wheel_speed_mps);
			}
		}

		// Match the PX4 update semantics: a sample skipped by the ratio is not
		// fused later. A future callback must provide a new sample.
		gps_measurement_.updated = false;
		gyro_measurement_.updated = false;
		wheel_speed_measurement_.updated = false;

		publishEstimate();
	}

	void publishEstimate()
	{
		if (!ekf_.initialized()) {
			return;
		}

		const auto stamp = now();
		msg::BetaEkfState state;
		state.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
		state.header.frame_id = odom_frame_;
		state.initialized = estimator_initialized_;
		state.vehicle_input_stale = !process_inputs_.received;
		state.gps_velocity_accepted = last_gps_accepted_;
		state.gyro_accepted = last_gyro_accepted_;
		state.wheel_speed_accepted = last_wheel_speed_accepted_;
		state.state = {{
			ekf_.posE(), ekf_.posN(), ekf_.psi(), ekf_.vx(), ekf_.vy(), ekf_.yawRate(),
			ekf_.rearWheelSpeed(), ekf_.deltaFxHat(), ekf_.deltaFyHat()
		}};
		state.variance = {{
			ekf_.varPosE(), ekf_.varPosN(), ekf_.varPsi(), ekf_.varVx(), ekf_.varVy(),
			ekf_.varYawRate(), ekf_.varRearWheelSpeed(), ekf_.varDeltaFxHat(), ekf_.varDeltaFyHat()
		}};
		state.beta_rad = ekf_.beta();
		state_publisher_->publish(state);

		nav_msgs::msg::Odometry odometry;
		odometry.header = state.header;
		odometry.child_frame_id = base_frame_;
		odometry.pose.pose.position.x = ekf_.posE();
		odometry.pose.pose.position.y = ekf_.posN();
		odometry.pose.pose.orientation.w = std::cos(0.5 * ekf_.psi());
		odometry.pose.pose.orientation.z = std::sin(0.5 * ekf_.psi());
		odometry.twist.twist.linear.x = ekf_.vx();
		odometry.twist.twist.linear.y = ekf_.vy();
		odometry.twist.twist.angular.z = ekf_.yawRate();
		odometry.pose.covariance[0] = ekf_.varPosE();
		odometry.pose.covariance[7] = ekf_.varPosN();
		odometry.pose.covariance[14] = kUnsupportedVariance;
		odometry.pose.covariance[21] = kUnsupportedVariance;
		odometry.pose.covariance[28] = kUnsupportedVariance;
		odometry.pose.covariance[35] = ekf_.varPsi();
		odometry.twist.covariance[0] = ekf_.varVx();
		odometry.twist.covariance[7] = ekf_.varVy();
		odometry.twist.covariance[14] = kUnsupportedVariance;
		odometry.twist.covariance[21] = kUnsupportedVariance;
		odometry.twist.covariance[28] = kUnsupportedVariance;
		odometry.twist.covariance[35] = ekf_.varYawRate();
		odometry_publisher_->publish(odometry);

		std_msgs::msg::Float32 beta;
		beta.data = ekf_.beta();
		beta_publisher_->publish(beta);
	}

	void resetCallback(
		const std::shared_ptr<std_srvs::srv::Trigger::Request>,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response)
	{
		reset_requested_ = true;
		response->success = true;
		response->message = gps_measurement_.valid
			? "EKF will reset from the latest odometry sample"
			: "Reset queued; waiting for the first odometry sample";
	}

	BetaEKF ekf_;
	ProcessInputs process_inputs_{};
	GpsVelocityMeasurement gps_measurement_{};
	GyroMeasurement gyro_measurement_{};
	WheelSpeedMeasurement wheel_speed_measurement_{};

	double loop_rate_hz_{200.0};
	double measurement_fuse_ratio_{1.0};
	double measurement_fuse_accumulator_{0.0};
	double reset_gap_s_{0.5};
	bool fuse_gps_velocity_{true};
	bool fuse_gyro_{true};
	bool fuse_wheel_speed_{true};
	bool estimator_initialized_{false};
	bool reset_requested_{false};
	bool last_gps_accepted_{false};
	bool last_gyro_accepted_{false};
	bool last_wheel_speed_accepted_{false};

	std::string odom_frame_;
	std::string base_frame_;
	std::string vehicle_state_topic_;
	std::string state_topic_;
	std::string odometry_output_topic_;
	std::string beta_topic_;

	std::chrono::steady_clock::time_point last_loop_time_{std::chrono::steady_clock::now()};

	rclcpp::Publisher<msg::BetaEkfState>::SharedPtr state_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr beta_publisher_;
	rclcpp::Subscription<bng_msgs::msg::BngVehicleStateMsg>::SharedPtr vehicle_state_subscription_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
	rclcpp::TimerBase::SharedPtr estimator_timer_;
};

}  // namespace beta_ekf_ros

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<beta_ekf_ros::BetaEkfNode>());
	rclcpp::shutdown();
	return 0;
}
