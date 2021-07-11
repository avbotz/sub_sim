/** @file state.hpp
 *  @brief State euler angles/quaternions, constant, and class definitions to use in the controller.
 *
 *  @author David Zhang
 *  @author Craig Wang
 */

#ifndef SIMULATOR_PIDCONTROLLER_HPP
#define SIMULATOR_PIDCONTROLLER_HPP 

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include "simulator/commandParser.hpp"

#define BB_TO_SIM "/tmp/bb_to_sim.txt"
#define SIM_TO_BB "/tmp/sim_to_bb.txt"

/*! @name Runtime configuration.
 */
///@{
/** Set to true when in a pool deep enough for the DVL to work (more than 5 ft).
 *
 *  You can set this to true even if Nautical is being compiled on land, but
 *  make sure the kill switch isn't set to alive until the sub is in the water.
 *  Otherwise, the entire program will hang and stop producing output. 
 */
static const bool DVL_ON = true;

/** Set to true when using the initial heading after the sub is unkilled. For
 *  example, if the diver unkills at 274.5 degrees, that will be treated as
 *  north for the sub during the run.
 */
static const bool USE_INITIAL_HEADING = true;

/** This isn't being used at the moment because the ahrs is not producing the
 *  same value after recompiling (not sure if this is supported, or if this is a
 *  bug in the code).
 *
 *  The idea was to have the sub use an absolute value for north rather than one
 *  relative to unkill. Since this would depend less on the diver, it would be
 *  more consistent. 
 */
static const bool FAR = true;

/** Use this when running this code on a personal computer rather than the sub.
 *  It will ignore most of the IO functions, unless you have access to a DVL and
 *  M5 motors (:P).
 */
static const bool SIM = true;
///@}

/*! @name Constants for degrees of freedom with North-East-Down coordinates. 
 *
 *  The degrees of freedom are X, Y, Z, Yaw, Pitch, Roll. The 7th degree of
 *  option is to replace Z (down) with depth from bottom instead.
 */
///@{
/** The number of degrees of freedom.
 */
static const int DOF = 6;

/** The number of  axes (North, East, Down, Yaw, Pitch, Roll).
 */
static const int N = 6;

/** The number of linear axes (North, East, Down).
 */
static const int BODY_DOF = 3;

/** The number of angular axes (Yaw, Pitch, Roll) plus the number of linear axes.
 */
static const int GYRO_DOF = 6;

/** Forward, or north, state index.
 */
static const int F = 0;

/** Horizontal, or east, state index.
 */
static const int H = 1;

/** Vertical, or down, state index.
 */
static const int V = 2;

/** Yaw state index.
 */
static const int Y = 3;

/** Pitch state index.
 */
static const int P = 4;

/** Roll state index.
 */
static const int R = 5;

/** Depth from bottom index.
 */
static const int D = 6;
///@}

/*! @name Motor ID configuration.
 */
///@{
/** Thruster enum makes code more readable. Do NOT set an ID to 0, the motor 
 *  will brick.
 */
enum thruster
{
	VERT_FL = 1,
	VERT_FR = 2,
	VERT_BL = 3,
	VERT_BR = 4,
	SURGE_FL = 5,
	SURGE_FR = 6,
	SURGE_BL = 7,
	SURGE_BR = 8,
	NUM_THRUSTERS
};

/** Number of motors.
 */
static const int NUM_MOTORS = 8;
///@}

/*! @name Motor orientation configuration.
 */
/** Rows are motors while columns are directions. Positive numbers are for 
 *  clockwise rotation and negative numbers are for counterclockwise.
 *
 *  The multipliers are either -1, 1, or 0 because they are meant to be
 *  direction only. I do set 1.1 for some motors when the sub is moving foward
 *  because it tends to drift right. If this code ever runs on new sub, which is
 *  supposed to correct for that, then it would be a good idea to change the
 *  values back to 1.
 */
static const float ORIENTATION[8][7] = 
{
	{ 0.00, 0.00, 1.00, 0.00, -1.0, -1.0, 1.00 },
	{ 0.00, 0.00, -1.0, 0.00, 1.00, -1.0, -1.0 },
	{ 0.00, 0.00, -1.0, 0.00, -1.0, 1.00, -1.0 },
	{ 0.00, 0.00, 1.00, 0.00, 1.00, 1.00, 1.00 },
	{ 1.00, 1.00, 0.00, 1.00, 0.00, 0.00, 0.00 },
	{ -1.0, 1.00, 0.00, 1.00, 0.00, 0.00, 0.00 },
	{ -1.0, 1.00, 0.00, -1.0, 0.00, 0.00, 0.00 }, 
	{ 1.00, 1.00, 0.00, -1.0, 0.00, 0.00, 0.00 }
};

/*! @name PID gains configuration.
 */
/** Rows correspond to F, H, V, Y, P, and R while columns correspond to kp, ki,
 *  and kd. To be honest, these gains are not great. The integral component is
 *  being ignored at the moment because it doesn't work as well in practice. 
 */
static const float GAINS[7][3] = 
{
	{ 2.00, 0.00, 8.50 },
	{ 2.00, 0.00, 8.50 },
	{ 12.0, 0.00, 10.0 },
	{ 1.00, 0.00, 1.00 },
	{ 1.00, 0.00, 1.00 },
	{ 0.10, 0.00, 0.05 },
	{ 0.75, 0.00, 2.50 }
};

// Down camera is 0.072 m in front of center of sub
static const float DOWN_CAM_X_OFFSET = 0.072;

// Down camera is 0.218 m left of center of sub
static const float DOWN_CAM_Y_OFFSET = -0.218;

// Ball dropper center is 0.089 m behind the down camera's center
static const float DROPPER_X_OFFSET = -0.089;

// Dropper center is 0.002 m to the right of the down camera's center
static const float DROPPER_Y_OFFSET = 0.002;

// 2 balls; one is 3 cm right of midline, other is 3 cm left of midline
static const float BALL_OFFSET[2] = { 0.03, -0.03 };

/*! @name Conversions.
 */
///@{
#define D2R 3.1415/180.
#define R2D 180./3.1415
///@}

/** @brief Helper class for PID computations.
 */
struct PID 
{
	/** PID gains for proportional, integral, and derivative. */
	float kp, ki, kd;

	/** Previous error value. */
	float prev;
	
	/** Sum of all of the errors. */
	float sum;

	PID() {}
	PID(float a, float b, float c) : kp(a), ki(b), kd(c), prev(0.), sum(0.) {}

	/** @brief Initialize the PID gains from the config.
	 *  
	 *  @param a Proportional gain.
	 *  @param b Integral gain.
	 *  @param c Derivative gain.
	 *  @return 0 on success.
	 */
	float init(float a, float b, float c);

	/** @brief Compute total PID constant.
	 *  
	 *  @param error Difference between setpoint and current point.
	 *  @param dt Time difference.
	 *  @param min Minimum value of PID, which is usefull to ensure small changes 
	 *             are adjusted for.
	 *  @return The total PID constant.
	 */
	float calculate(float error, float dt, float min);
};

class PIDController 
{
	public:
		float INITIAL_YAW, INITIAL_PITCH, INITIAL_ROLL,
			altitude, desired_altitude, daltitude, 
			state[N], pose[DOF], current[DOF], desired[DOF], 
			dstate[DOF], thrust[NUM_MOTORS], pid[DOF], 
			mtime, ktime, vel_x, vel_y, p;

		bool alreadyDroppedBall_0, alreadyDroppedBall_1, initial_yaw_set;

		// Some max effort multipliers are negative because uuv propellers spin in opposite direction, so we need to reverse our input 
		// to get to the same destination.
		const float max_efforts[NUM_MOTORS] = {-2000., 2000., 2000., -2000., -2000., 2000., 2000., -2000.};

		PID controllers[DOF+1];
		commandParser Serial;
		FILE* out;

		ros::NodeHandle node;
		ros::Subscriber state_subscriber, setpoint_subscriber;
		ros::ServiceClient gms;
		std::vector<ros::Publisher> thrust_pubs;

		gazebo_msgs::GetModelState model_state;

		PIDController();
		void run();
		void getPorpoiseCommand();
		bool alive();
		void kalmanCompute();
		void publishThrusterEfforts();
		void drop(int, int);
		void getDvlMessageCallback(const uuv_sensor_ros_plugins_msgs::DVL &);
		void getImuMessageCallback(const sensor_msgs::Imu::ConstPtr &);
		void getPressureMessageCallback(const sensor_msgs::FluidPressure::ConstPtr &);
};

#endif