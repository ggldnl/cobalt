// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/motor/motor.h"

/**
 * @brief Split the string based on the separator c
 *
 * @param str : string to split
 * @param c : separator
 * @return std::vector<std::string> : tokens
 */
std::vector<std::string> split(const char *str, char c = ' ') {
	std::vector<std::string> result;

	do {
		const char *begin = str;

		while(*str != c && *str)
			str++;

		result.push_back(std::string(begin, str));
	} while (0 != *str++);

	return result;
}

#define IN_1_LEFT 25
#define IN_2_LEFT 24
#define ENCODER_LEFT 0
#define ENCODER_RIGHT 2
#define IN_1_RIGHT 23
#define IN_2_RIGHT 22
#define ENABLE 29

Motor left_motor (ENABLE, IN_1_LEFT, IN_2_LEFT, ENCODER_LEFT);
Motor right_motor (ENABLE, IN_1_RIGHT, IN_2_RIGHT, ENCODER_RIGHT);

// sloppy input handling ¯\_( ͡° ͜ʖ ͡°)_/¯ 
void callBack (const std_msgs::String::ConstPtr& msg) {

	/*
	 * request format: <motor> (<direction> <speed> | "stop") | "stop"
	 * e.g. 
	 * 		stop				 	// stop
	 *		forward <speed>		 	// <direction> <speed>
	 * 		backward <speed>	 	// <direction> <speed>
	 *		left stop			 	// left stop
	 *		left forward <speed> 	// left <direction> <speed>
	 * 		left backward <peed> 	// left <direction> <speed>
	 *		right stop			 	// right stop
	 *		right forward <speed> 	// right <direction> <speed>
	 * 		right backward <peed> 	// right <direction> <speed>
	 *
	 *		stop
	 *		<direction> <speed>
	 *		<motor> stop
	 *		<motor> <direction> <speed>
	 */
	ROS_INFO("Command received: %s", msg -> data.c_str());

	std::vector<std::string> tokens = split(msg -> data.c_str());
	std::string token = tokens[0];

	// command: stop

	if (tokens.size() == 1) {
		// command is "stop"
		if (token == "stop") {
			left_motor.stop();
			right_motor.stop();
		} else
			ROS_INFO("Invalid command %s", msg -> data.c_str());
	}

	// command: <direction> <speed>

	if (tokens.size() == 2) {

		// command could be "right" (right stop), "left" (left stop),
		// 		"forward" (forward <speed>), "backward" (backward <speed>)

		if (token == "right") {

			if (tokens[1] == "stop")
				right_motor.stop();
			else
				ROS_INFO("Invalid command %s", msg -> data.c_str());

		} else if (token == "left") {

			if (tokens[1] == "stop")
				left_motor.stop();
			else
				ROS_INFO("Invalid command %s", msg -> data.c_str());
		
		} else if (token == "forward" || token == "backward") {

			int speed = stoi(tokens[1]);
			if (token == "forward") {
				right_motor.forward(speed);
				left_motor.forward(speed);
			}
			else {
				right_motor.backward(speed);
				left_motor.backward(speed);
			}

		} else {
			ROS_INFO("Invalid command %s", msg -> data.c_str());
		}
	}

	// command: <motor> <direction> <speed>

	if (tokens.size() == 3) {
		std::string motor = tokens[0];
		std::string direction = tokens[1];
		int speed = stoi(tokens[2]);		
		
		if (motor == "left") {
			
			if (direction == "forward") {
				left_motor.forward(speed);
			} else if (direction == "backward") {
				left_motor.backward(speed);
			} else {
				ROS_INFO("Invalid command %s", msg -> data.c_str());
			}

		} else if (motor == "right") {

			if (direction == "forward") {
				right_motor.forward(speed);
			} else if (direction == "backward") {
				right_motor.backward(speed);
			} else {
				ROS_INFO("Invalid command %s", msg -> data.c_str());
			}

		} else {
			ROS_INFO("Invalid command %s", msg -> data.c_str());
		}
	}

	if (tokens.size() > 3)
		ROS_INFO("Invalid command %s", msg -> data.c_str());
}

int main(int argc, char **argv) {

	// wiringPiSetupGpio(); // Broadcom GPIO
	wiringPiSetup(); // wPi

	left_motor.setup();
	right_motor.setup();

	ros::init(argc, argv, "motor_test");
	ros::NodeHandle nh;

	/*
	 * subscribe to "motor_command" topic (std_msgs::String)
	 *      format: ( <direction> <speed> | "stop")
	 *      example: forward 50
	 *      example: stop
	 *      example: left 50
	 */

	ros::Subscriber command_sub = nh.subscribe("motor_command", 1000, callBack);

	// 0.2 Hz -> one event each 5 seconds 1/5 = 0.2
	// 0.5 Hz -> one event each 2 seconds 1/2 = 0.5
	ros::Rate loop_rate(0.5);

	ROS_INFO("Initialization complete");

	while (ros::ok()) {

		ros::spinOnce();

		loop_rate.sleep(); // sleep for 1 sec since we are at 1Hz
	}

	left_motor.stop();
	right_motor.stop();

	return 0;
}