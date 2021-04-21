/** @file main.cpp
 *  @brief Main code that controls the simulated Nemo's thrusters.
 *
 *  @author Craig Wang
 */
#include <ros/package.h>
#include "simulator/PIDController.hpp"

int main(int argc, char** argv)
{
	// Initialize node and thruster controller.
	ros::init(argc, argv, "simulator_node");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	PIDController controller;

	// Set up subscribers to read simulated sensors.
	ros::Subscriber dvl_subscriber = 
		node.subscribe("/nemo/dvl", 1, &PIDController::getDvlMessageCallback, &controller);
	ros::Subscriber imu_subscriber = 
		node.subscribe("/nemo/imu", 1, &PIDController::getImuMessageCallback, &controller);
	ros::Subscriber pressure_subscriber = 
		node.subscribe("/nemo/pressure", 1, &PIDController::getPressureMessageCallback, &controller);

	// Control the simulated sub's thrusters.
	while (ros::ok())
	{
		ros::spinOnce();
		controller.run();
		loop_rate.sleep();
	}
}