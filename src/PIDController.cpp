/** @file nautical_controller.cpp
 *  @brief PID controller class for UUV Simulator, emulating Nautical's controller.
 *
 *  @author David Zhang
 *  @author Craig Wang
 */
#include "simulator/PIDController.hpp"
#include "simulator/commandParser.hpp"
#include "simulator/utils.hpp"

PIDController::PIDController()
{
	/* Subscribers must be set up outside of the class */
	
	// Start off with clean variables of 0
	for (int i = 0; i < DOF; i++) this->state[i] = 0.;
	for (int i = 0; i < DOF; i++) this->pose[i] = 0.;
	for (int i = 0; i < DOF; i++) this->current[i] = 0.;
	for (int i = 0; i < DOF; i++) this->desired[i] = 0.;
	for (int i = 0; i < DOF; i++) this->dstate[i] = 0.;
	for (int i = 0; i < NUM_MOTORS; i++) this->thrust[i] = 0.;
	for (int i = 0; i < DOF; i++) this->controllers[i].init(GAINS[i][0], GAINS[i][1], GAINS[i][2]);

	this->controllers[D].init(GAINS[D][0], GAINS[D][1], GAINS[D][2]);
	this->p = 0.;
	this->altitude = 0.;
	this->desired_altitude = -1.;
	this->daltitude = 0.;
	this->mtime = ros::Time::now().toSec();
	this->ktime = ros::Time::now().toSec();
	this->vel_x = 0.;
	this->vel_y = 0.;

	// Initialize mock serial port to communicate with Porpoise
	this->Serial.init(BB_TO_SIM);
	this->out = fopen(SIM_TO_BB, "w+");

	// State data from gazebo is already updated, set these for the initial values
	if (USE_INITIAL_HEADING)
		this->INITIAL_YAW = this->pose[Y];
	else
		INITIAL_YAW = FAR ? 225. : 340.;
	this->INITIAL_PITCH = this->pose[P];
	this->INITIAL_ROLL = this->pose[R];

	// Set up gazebo pose listener
	this->gms = this->node.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	this->model_state.request.model_name = "nemo";

	// Initialize ball dropping status
	this->alreadyDroppedBall_0 = false;
	this->alreadyDroppedBall_1 = false;

	// Set up thruster effort publishers
	ros::Publisher thr0_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/0/input", 1);
	ros::Publisher thr1_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/1/input", 1);
	ros::Publisher thr2_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/2/input", 1);
	ros::Publisher thr3_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/3/input", 1);
	ros::Publisher thr4_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/4/input", 1);
	ros::Publisher thr5_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/5/input", 1);
	ros::Publisher thr6_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/6/input", 1);
	ros::Publisher thr7_pub = 
		this->node.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/nemo/thrusters/7/input", 1);

	this->thrust_pubs.insert(this->thrust_pubs.end(), { 
		thr0_pub, thr1_pub, thr2_pub, thr3_pub,
		thr4_pub, thr5_pub, thr6_pub, thr7_pub
	});
}

void PIDController::run()
{
	// Motors are done starting up and the sub is alive. Run the sub as
	// intended.
	this->getPorpoiseCommand();
	this->current[V] = this->pose[V];
	this->current[Y] = this->pose[Y] - INITIAL_YAW;
	// current[P] = ahrs_att((enum att_axis) (PITCH)) - INITIAL_PITCH;
	// current[R] = ahrs_att((enum att_axis) (ROLL)) - INITIAL_ROLL;
	this->current[P] = this->pose[P];
	this->current[R] = this->pose[R];

	// Distance to bottom already taken care of in getDvlMessageCallback()

	// Handle angle overflow/underflow.
	for (int i = BODY_DOF; i < GYRO_DOF; i++)
		this->current[i] += (this->current[i] > 180.) ? -360. : (this->current[i] < -180.) ? 360. : 0.;
	// float temp[3] = { current[Y], current[P], current[R] };

	// Kalman filter removes noise from measurements and estimates the new
	// state. Assume angle is 100% correct so no need for EKF or UKF.
	this->kalmanCompute();

	// Use KF for N and E components of state. 
	// this->current[F] = state[0];
	// this->current[H] = state[3];

	// Change heading if desired state is far. Turned off for now
	// because we want to rely on DVL > AHRS.
	/*
	if (fabs(desired[F]-current[F]) > 3. || fabs(desired[H]-current[H] > 3.))
		desired[Y] = atan2(desired[H]-current[H], desired[F]-current[F]) * R2D;
	desired[Y] += (desired[Y] > 360.) ? -360. : (desired[Y] < 0.) ? 360. : 0.;
	*/

	// Compute the state difference. Change heading first if the heading
	// error is high. Make depth changes regardless. 
	for (int i = 0; i < DOF; i++)
		this->dstate[i] = 0.;
	float d0 = this->desired[F] - this->current[F];
	float d1 = this->desired[H] - this->current[H];
	float i0 = d0*cos(D2R*this->current[Y]) + d1*sin(D2R*this->current[Y]);
	float i1 = d1*cos(D2R*this->current[Y]) - d0*sin(D2R*this->current[Y]);
	if (fabs(angle_difference(desired[Y], current[Y])) > 5.)
	{
		this->dstate[Y] = angle_difference(this->desired[Y], this->current[Y]);
	}
	else if (fabs(i1) > 2.)
	{
		this->dstate[Y] = angle_difference(this->desired[Y], this->current[Y]);
		this->dstate[F] = i0 < i1/3. ? i0 : i1/3.;
		this->dstate[H] = i1; 
	}
	else 
	{
		this->dstate[Y] = angle_difference(this->desired[Y], this->current[Y]);
		this->dstate[F] = i0;
		this->dstate[H] = i1;
	}

	this->dstate[V] = this->desired[V] - this->current[V];
	this->daltitude = this->desired_altitude > 0. ? this->desired_altitude - this->altitude : -9999.;

	// Compute PID within motors and set thrust.
	// mtime = motors.run(dstate, daltitude, temp, mtime);

	// Calculate time difference since last iteration.
	// uint32_t temp = micros();
	// float dt = (temp - t)/(float)(1000000);
	float t = ros::Time::now().toSec();
	float dt = t - this->mtime;
	if (dt == 0.) return;

	// Calculate PID values. Third argument is minimum PID value, which allows
	// changes for small values, though it doesn't seem to affect the code for
	// now.
	this->pid[F] = this->controllers[F].calculate(this->dstate[F], dt, 0.20);
	this->pid[H] = this->controllers[H].calculate(this->dstate[H], dt, 0.20);
	for (int i = BODY_DOF; i < GYRO_DOF; i++)
		this->pid[i] = this->controllers[i].calculate(this->dstate[i], dt, 0.00);

	// Choose between depth from bottom or depth sensor. 
	if (this->daltitude < -999.)
	{
		this->pid[V] = this->controllers[V].calculate(this->dstate[V], dt, 0.00);
		// Serial << dstate[V] << " " << pid[V] << '\n';
	}
	else 
	{
		this->pid[V] = -1.*this->controllers[D].calculate(this->daltitude, dt, 0.00);
		// Serial << daltitude << " " << pid[V] << '\n';
	}

	// Default motor thrusts are 0. Add 0.15 to vertical thrusts so the sub
	// remains at the same depth.
	for (int i = 0; i < NUM_MOTORS; i++)
		this->thrust[i] = 0.;

	if (this->p > 0.01)
	{
		this->thrust[0] += 0.15;
		this->thrust[1] -= 0.15;
		this->thrust[2] -= 0.15;
		this->thrust[3] += 0.15;
	}

	// Compute final thrust given to each motor based on orientation matrix and
	// PID values.
	for (int i = 0; i < NUM_MOTORS; i++)
		for (int j = 0; j < DOF; j++) 
			this->thrust[i] += this->p*this->pid[j]*ORIENTATION[i][j];
	//if (!SIM) power();

	this->mtime = ros::Time::now().toSec();
	this->publishThrusterEfforts();
}

void PIDController::getPorpoiseCommand()
{
	/* Read Porpoise command */
	if (this->Serial.available() > 0)
	{
		char c = this->Serial.read();

		if (c == 'a')
		{
			fprintf(this->out, "%d\n", this->alive());
			fflush(this->out);
		}
		else if (c == 'c')
		{
			fprintf(this->out, "%f %f %f %f %f %f\n",
				this->current[F], this->current[H], this->current[V],
				this->current[Y], this->current[P], this->current[R]);
			fflush(this->out);
		}
		else if (c == 'p')
		{
			this->p = this->Serial.parseFloat();
		}
		else if (c == 's')
		{
			for (int i = 0; i < DOF; i++)
				this->desired[i] = this->Serial.parseFloat();
		}
		else if (c == 'z')
		{
			this->desired_altitude = this->Serial.parseFloat();
		}
		else if (c == 'w')
		{
			fprintf(this->out, "%f\n", this->altitude);
			fflush(this->out);
		}
		else if (c == 'r')
		{
			for (int i = 0; i < DOF; i++)
				this->desired[i] = this->current[i];
			float temp1[3];
			float temp2[3] = {this->current[Y], this->current[P], this->current[R]};
			for (int i = 0; i < BODY_DOF; i++)
				temp1[i] = this->Serial.parseFloat();
			float temp[3];
			body_to_inertial(temp1, temp2, temp);
			for (int i = 0; i < BODY_DOF; i++)
				this->desired[i] += temp[i];
			for (int i = BODY_DOF; i < GYRO_DOF; i++)
				this->desired[i] = angle_add(this->current[i], this->Serial.parseFloat());
		}
		else if (c == 'g')
		{
			int idx = this->Serial.parseInt();
			int val = this->Serial.parseInt();
			this->drop(idx, val);
		}
		else if (c == 'u')
		{
			int dof = this->Serial.parseInt();
			float gains[3];
			for (int i = 0; i < 3; i++)
				gains[i] = this->Serial.parseFloat();
			this->controllers[dof].init(gains[0], gains[1], gains[2]);
		}
	}
}

bool PIDController::alive()
{
	/* Checks if simulator has loaded up */
	if (ros::Time::now().toSec() > 15.) 
		return true;
	else 
		return false;
}

void PIDController::kalmanCompute()
{
	/* Get current sub state using DVL velocity data */

	// Calculate time difference since last iteration.
	float t = ros::Time::now().toSec();
	float dt = t - this->ktime;

	// Convert relative velocity to absolute state
	float input[3] = {this->vel_x, this->vel_y, 0.};
	float angles[3] = {this->current[Y], this->current[P], this->current[R]};
	float output[3];
	body_to_inertial(input, angles, output);
	this->current[F] += output[F] * dt;
	this->current[H] += output[H] * dt;

	// Update time of last iteration
	this->ktime = ros::Time::now().toSec();
}

void PIDController::publishThrusterEfforts()
{
	/*
	 * Expects that thrust input is between -1 and 1.
	 */
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		uuv_gazebo_ros_plugins_msgs::FloatStamped msg;
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = std::string("nemo/thruster_") + std::to_string(i);
		msg.data = this->thrust[i] * this->max_efforts[i];
		this->thrust_pubs[i].publish(msg);
	}
}

void PIDController::drop(int idx, int val)
{
	/* Val of 1 = open, 0 = close */

	// Prevent ball from continually teleporting back up to sub
	if ((idx == 0 && this->alreadyDroppedBall_0) || (idx == 1 && this->alreadyDroppedBall_1) || (val == 0)) 
		return;

	// Set up ros services to use, use Gazebo's state because it is more accurate than the DVL
	// And, the DVL is not located at the center of the sub, so more changes would be needed if we used this->current.
	ros::NodeHandle node;
	ros::ServiceClient gms = node.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	ros::ServiceClient sms = node.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	gazebo_msgs::GetModelState getModelState;
	gazebo_msgs::SetModelState setModelState;
	gazebo_msgs::ModelState ball;
	gms.waitForExistence();
	sms.waitForExistence();

	// Find Nemo position to know where to teleport ball
	getModelState.request.model_name = "nemo";
	gms.call(getModelState);

	// Convert relative to absolute coordinates to teleport ball to
	float input[3] = {DOWN_CAM_X_OFFSET + DROPPER_X_OFFSET, 
		DOWN_CAM_Y_OFFSET + DROPPER_Y_OFFSET + BALL_OFFSET[idx], 0.4};
	float angles[3] = {this->current[Y], this->current[P], this->current[R]};
	float output[3];
	body_to_inertial(input, angles, output);

	// Convert North-East-Down state to East-North-Up state that Gazebo uses
	double x = getModelState.response.pose.position.x + output[F];
	double y = getModelState.response.pose.position.y - output[H];
	double z = getModelState.response.pose.position.z - output[V];

	// Teleport ball to sub where it then drops
	ball.model_name = "ball_" + std::to_string(idx);
	ball.pose.position.x = x;
	ball.pose.position.y = y;
	ball.pose.position.z = z;
	ball.pose.orientation.x = 1.;
	ball.pose.orientation.y = 0.;
	ball.pose.orientation.z = 0.;
	ball.pose.orientation.w = 0.;

	setModelState.request.model_state = ball;
	sms.call(setModelState);

	if (idx == 0) this->alreadyDroppedBall_0 = true;
	else if (idx == 1) this->alreadyDroppedBall_1 = true;
}

void PIDController::getDvlMessageCallback(const uuv_sensor_ros_plugins_msgs::DVL &msg)
{
	/*
	 * Get current altitude and velocity using DVL sensor
	 */

	// The DVL gets velocity pointed downwards in ENU system, so convert it to reference frame of sub
	this->altitude = msg.altitude;
	this->vel_x = msg.velocity.z;
	this->vel_y = -msg.velocity.y;
}

void PIDController::getImuMessageCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	/*
	 * Get current angles using IMU sensor
	 */

	// Get angles in quaternion format
	double qx = msg->orientation.x;
	double qy = msg->orientation.y;
	double qz = msg->orientation.z;
	double qw = msg->orientation.w;

	// Convert ENU Quaternions to NED Euler degrees
	Quaternion q;
	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;

	EulerAngles angles = ToEulerAngles(q);
	angles.yaw *= R2D;
	angles.pitch *= R2D;
	angles.roll *= R2D;

	// Update current angles
	if (isValidValue(-angles.yaw) &&
		isValidValue(-angles.pitch) &&
		isValidValue(angles.roll))
	{
		this->pose[Y] = -angles.yaw;
		this->pose[P] = -angles.pitch;
		this->pose[R] = angles.roll;
	}
}

void PIDController::getPressureMessageCallback(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
	/*
	 * Get current depth using pressure sensor
	 */
	float depth = (msg->fluid_pressure - 101.325) / 9.80638;
	if (isValidValue(depth))
		this->pose[V] = depth;
}

float PID::init(float a, float b, float c)
{
	this->kp = a;
	this->ki = b;
	this->kd = c;
	this->prev = 0.0;
	this->sum = 0.0;
}

/*
 * IMPORTANT: Make sure DT isn't 0, will cause motors to send NAN.
 */
float PID::calculate(float error, float dt, float min)
{
	float pout = this->kp*error;
	this->sum += error*dt;
	float iout = this->ki*this->sum;
	float dout = this->kd*(error-this->prev)/dt;
	this->prev = error;
	float output = pout + iout + dout;
	float dir = 1.;
	if (output < 0) dir = -1.;

	float out = limit(limit(output, -2., 2.), min);
	return out;
}