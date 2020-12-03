cmake_minimum_required(VERSION 3.0.2)
project(simulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	genmsg
	uuv_gazebo_ros_plugins_msgs
	message_generation)

catkin_package(CATKIN_DEPENDS std_msgs
               INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simulator_node src/simulator.cpp src/PIDController.cpp 
	src/commandParser.cpp src/utils.cpp)
	
target_link_libraries(simulator_node ${catkin_LIBRARIES})

catkin_package(INCLUDE_DIRS include LIBRARIES simulator)

install(DIRECTORY launch worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_package()
