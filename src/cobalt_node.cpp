/**
 * @file cobalt.cpp
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief
 * 	
 * Main ROS node
 *
 * subscribing to:
 * "/cmd_vel" geometry_msgs/Twist
 * "/robot_pid" std_msgs/String
 * 
 * Using only standard messages it is not mandatory to compile the same package on 
 * both the master and the slave in case they are hosted on two separate machines
 * 
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// ros libs
#include <ros/ros.h>

// my libraries (the node only knows about the robot interface)
// #include "../include/robot.h"
// #include "../include/utils.h"

/*
 * Robot interface: exports methods to control the robot
 */
// Robot robot;

int update_frequency = 1; // 1Hz, 1 event each second, both for position and info

int main (int argc, char ** argv) {

	/*
	 * Setup the hardware. The robot will enable the drv8833 and recursively call
	 * the same method on the motors that will initialize their pins and such
	 */
	// robot.setup();
	// robot.left_motor.pid.set_manual();
	// robot.right_motor.pid.set_manual();

	/*
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it. The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 *
	 * We will have a launch file that will launches this node and the raspicam_node
	 */
	ros::init(argc, argv, "cobalt");
	ros::NodeHandle nh;

    // TODO declare publishers and subscribers

	/**
	 * A ros::Rate object allows you to specify a frequency that you would
	 * like to loop at. It will keep track of how long it has been since the
	 * last call to Rate::sleep(), and sleep for the correct amount of time.
	 */
	ros::Rate loop_rate(update_frequency);
	// ros::Rate loop_rate(10); // 10 Hz -> 10 times a second

	while (ros::ok()) {

		/*
		 * ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by the master.
		 */
		ros::spinOnce();

		// format position message

        // send position message

        // send misc info

		loop_rate.sleep();
	}

	// ros is not ok anymore :(
	// robot.stop();

	return 0;
}

