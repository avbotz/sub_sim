/** @file utils.hpp
 *  @brief Declarations for functions to be used in the simulator controller.
 *
 *  @author David Zhang
 *  @author Craig Wang
 */

#ifndef SIMULATOR_UTILS_HPP
#define SIMULATOR_UTILS_HPP

#include <iostream>
#include <cmath>

struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles 
{
    double roll, pitch, yaw;
};

float angle_difference(float, float);
float angle_add(float, float);
float limit(float, float, float);
float limit(float, float);
bool isValidValue(float);
void drop(int, int);
void body_to_inertial(float *, float *, float *);
EulerAngles ToEulerAngles(Quaternion);

#define D2R 3.1415/180.
#define R2D 180./3.1415

#endif