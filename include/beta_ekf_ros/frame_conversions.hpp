#pragma once

#include <array>
#include <cmath>

namespace beta_ekf_ros::frames
{

using Quaternion = std::array<double, 4>;  // [w, x, y, z]
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;

inline bool finite(const Quaternion &q)
{
	return std::isfinite(q[0]) && std::isfinite(q[1]) &&
	       std::isfinite(q[2]) && std::isfinite(q[3]);
}

inline bool normalize(Quaternion &q)
{
	if (!finite(q)) {
		return false;
	}

	const double norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

	if (!std::isfinite(norm_sq) || norm_sq < 1e-12) {
		return false;
	}

	const double inverse_norm = 1.0 / std::sqrt(norm_sq);

	for (double &component : q) {
		component *= inverse_norm;
	}

	return true;
}

inline Matrix3 quaternionToMatrix(const Quaternion &q)
{
	const double w = q[0];
	const double x = q[1];
	const double y = q[2];
	const double z = q[3];

	return {{
		{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)}},
		{{2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)}},
		{{2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)}}
	}};
}

inline Matrix3 multiply(const Matrix3 &a, const Matrix3 &b)
{
	Matrix3 result{};

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			for (int index = 0; index < 3; ++index) {
				result[row][column] += a[row][index] * b[index][column];
			}
		}

	}

	return result;
}

inline Vector3 multiply(const Matrix3 &matrix, const Vector3 &vector)
{
	Vector3 result{};

	for (int row = 0; row < 3; ++row) {
		result[row] = matrix[row][0] * vector[0] +
			      matrix[row][1] * vector[1] +
			      matrix[row][2] * vector[2];
	}

	return result;
}

inline Quaternion matrixToQuaternion(const Matrix3 &matrix)
{
	Quaternion q{};
	const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];

	if (trace > 0.0) {
		const double scale = 2.0 * std::sqrt(trace + 1.0);
		q[0] = 0.25 * scale;
		q[1] = (matrix[2][1] - matrix[1][2]) / scale;
		q[2] = (matrix[0][2] - matrix[2][0]) / scale;
		q[3] = (matrix[1][0] - matrix[0][1]) / scale;

	} else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
		const double scale = 2.0 * std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
		q[0] = (matrix[2][1] - matrix[1][2]) / scale;
		q[1] = 0.25 * scale;
		q[2] = (matrix[0][1] + matrix[1][0]) / scale;
		q[3] = (matrix[0][2] + matrix[2][0]) / scale;

	} else if (matrix[1][1] > matrix[2][2]) {
		const double scale = 2.0 * std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
		q[0] = (matrix[0][2] - matrix[2][0]) / scale;
		q[1] = (matrix[0][1] + matrix[1][0]) / scale;
		q[2] = 0.25 * scale;
		q[3] = (matrix[1][2] + matrix[2][1]) / scale;

	} else {
		const double scale = 2.0 * std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
		q[0] = (matrix[1][0] - matrix[0][1]) / scale;
		q[1] = (matrix[0][2] + matrix[2][0]) / scale;
		q[2] = (matrix[1][2] + matrix[2][1]) / scale;
		q[3] = 0.25 * scale;
	}

	normalize(q);
	return q;
}

inline const Matrix3 &nedFromEnu()
{
	static const Matrix3 transform{{
		{{0.0, 1.0, 0.0}},
		{{1.0, 0.0, 0.0}},
		{{0.0, 0.0, -1.0}}
	}};
	return transform;
}

inline const Matrix3 &fluFromFrd()
{
	static const Matrix3 transform{{
		{{1.0, 0.0, 0.0}},
		{{0.0, -1.0, 0.0}},
		{{0.0, 0.0, -1.0}}
	}};
	return transform;
}

inline Quaternion rosFluEnuToCoreFrdNed(const Quaternion &ros_quaternion)
{
	const Matrix3 enu_from_flu = quaternionToMatrix(ros_quaternion);
	const Matrix3 ned_from_frd = multiply(multiply(nedFromEnu(), enu_from_flu), fluFromFrd());
	return matrixToQuaternion(ned_from_frd);
}

inline Vector3 bodyFluToWorldNed(const Quaternion &ros_quaternion, const Vector3 &body_flu)
{
	const Vector3 world_enu = multiply(quaternionToMatrix(ros_quaternion), body_flu);
	return multiply(nedFromEnu(), world_enu);
}

inline Vector3 fluToFrd(const Vector3 &flu)
{
	return multiply(fluFromFrd(), flu);
}

inline double yawFromRosQuaternion(const Quaternion &q)
{
	return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
			  1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

}  // namespace beta_ekf_ros::frames
