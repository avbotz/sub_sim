/** @file main.cpp
 *  @brief Main code that controls the simulated Nemo's thrusters.
 *
 *  @author Craig Wang
 */
#include "simulator/PIDController.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simulator_node");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);

	PIDController controller;
	controller.init();

	// Set up subscribers to receive commands from Porpoise and read simulated sensors
	ros::Subscriber command_subscriber = 
		node.subscribe("/nemo/commands", 1, &PIDController::getCommandCallback, &controller);
	ros::Subscriber dvl_subscriber = 
		node.subscribe("/nemo/dvl", 1, &PIDController::getDvlMessageCallback, &controller);
	ros::Subscriber imu_subscriber = 
		node.subscribe("/nemo/imu", 1, &PIDController::getImuMessageCallback, &controller);
	ros::Subscriber pressure_subscriber = 
		node.subscribe("/nemo/pressure", 1, &PIDController::getPressureMessageCallback, &controller);

	while (ros::ok())
	{
		ros::spinOnce();
		controller.run();
		loop_rate.sleep();
	}
}