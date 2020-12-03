/** @file main.cpp
 *  @brief Main code that runs the simulated Nemo's thrusters.
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

	ros::Subscriber command_subscriber = 
		node.subscribe("/nemo/commands", 1, &PIDController::getCommandCallback, &controller);

	while (ros::ok())
	{
		ros::spinOnce();
		controller.run();
		loop_rate.sleep();
	}
}