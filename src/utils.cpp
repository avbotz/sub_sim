/** @file utils.cpp
 *  @brief Helper functions to be used in the simulator controller.
 *
 *  @author David Zhang
 *  @author Craig Wang
 */
#include "simulator/utils.hpp"

float angle_difference(float a1, float a2)
{
	// For [-180, 180].
	float b1 = a1-a2;
	if (fabs(b1) > 180.)
	{
		if (a1 < a2)
			a1 += 360.;
		else 
			a2 += 360.;
		b1 = a1-a2;
	}
	return b1;
}

float angle_add(float a1, float add)
{
	float temp = a1 + add;
	if (temp > 180.0)
		return temp - 360.0;
	else if (temp < -180.)
		return temp + 360.0;

	return temp;
}

float limit(float input, float lower, float upper)
{
	if (input < lower) 
		return lower;
	if (input > upper)
		return upper;
	return input;
}

float limit(float input, float min)
{
	if (input < 0. && input > -1.*min)
		return -1*min;
	if (input > 0. && input < min)
		return min;
	return input;
}

bool isValidValue(float value)
{
	/* Checks if the number is a nan or too large */
	if (std::isnan(value) || fabs(value) > 1000.)
		return false;
	else 
		return true;
}

void body_to_inertial(float *input, float *angles, float *output)
{
	// Pre-compute trig functions.
	float spsi = std::sin(angles[0]*D2R);
	float sthe = std::sin(angles[1]*D2R);
	float sphi = std::sin(angles[2]*D2R);
	float cpsi = std::cos(angles[0]*D2R);
	float cthe = std::cos(angles[1]*D2R);
	float cphi = std::cos(angles[2]*D2R);

	// Calculate rotation matrix for Euler transformation.
	float r11 =  cthe*cpsi;
	float r12 = -cphi*spsi + sphi*sthe*cpsi;
	float r13 = -sphi*spsi + cphi*sthe*cpsi;
	float r21 =  cthe*spsi;
	float r22 =  cphi*cpsi + sphi*sthe*spsi;
	float r23 =  sphi*cpsi + cphi*sthe*spsi;
	float r31 = -sthe;
	float r32 =  sphi*cthe;
	float r33 =  cphi*cthe;

	// Calculate matrix.
	output[0] = r11*input[0] + r12*input[1] + r13*input[2];
	output[1] = r21*input[0] + r22*input[1] + r23*input[2];
	output[2] = r31*input[0] + r32*input[1] + r33*input[2];
}


EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}