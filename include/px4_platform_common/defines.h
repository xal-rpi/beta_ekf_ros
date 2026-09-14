#pragma once

#include <cmath>

inline bool PX4_ISFINITE(float value)
{
	return std::isfinite(value);
}

inline bool PX4_ISFINITE(double value)
{
	return std::isfinite(value);
}

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif
