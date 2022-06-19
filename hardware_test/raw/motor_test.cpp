// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// generic includes
#include <stdint.h>
#include <vector>
#include <string>

#include <wiringPi.h>
#include <softPwm.h>

// pins

#define IN_1_LEFT 25
#define IN_2_LEFT 24
#define IN_1_RIGHT 23
#define IN_2_RIGHT 22
#define ENABLE 29

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

/**
 * @brief Clamps a value between upper and lower bound.
 * If we set a pwm value of 70 it means that the signal will be high for 70% of the time
 * and low for the remaining 30. This brings the motor to have a speed of 30% (drv8833).
 * To avoid this, we invert the speed argument: speed -> 100 - speed. In addition to this
 * the initial value must be between 0 and 100, at least theoretically: from the observations
 * we get that with a speed = 10 the wheel doesn't even move. We can set the limits between
 * 20 and 100 (operative limit).
 *
 * @tparam T
 * @param v : value to clamp between upper and lower bound
 * @param bottom : lower bound
 * @param top : upper bound
 * @return T : new constrained value
 */
template <typename T>
static T clamp(T v, T bottom, T top){
	if (v > top)
		return top;
	if (v < bottom)
		return bottom;
	return v;
}

/**
 * @brief Filters the input speed value.
 * Whenever we try to set a particular speed, we use a 0-100 value
 * where 0 means do not move and 100 means move at full speed. These
 * values needs to be mapped in order for the DRV8833 to respond
 * correctly (100 - input_value). Furthermore, I empirically noticed
 * that when a pwm value below a certain threshold is applied, the
 * motor struggles too much to move; to avoid that, we block certain
 * pwm values (~ percentage values accepted as valid input); this
 * behaviour can vary depending on the motor you use
 *
 * @param speed : required speed
 * @return int : new, filtered, speed value for the microcontroller;
 * the user has no clue that the value used by the microcontroller is
 * different than that he passed
 */
int filter (int speed) {

	// TODO fix threshold outside the method
	int threshold = 20; // assert threshold is less than 50 at least

	int filtered_speed = clamp<int>(speed, threshold, 100);
	return 100 - filtered_speed;
}

void hardware_setup (void) {

	#ifdef __arm__

		// wiringPiSetupGpio(); // Broadcom GPIO
		wiringPiSetup(); // wPi

		// enable the DRV8833
		pinMode(ENABLE, OUTPUT);
		digitalWrite(ENABLE, HIGH);

		// set the pins as PWM output
		softPwmCreate(IN_1_LEFT, 0, 100);
		softPwmCreate(IN_2_LEFT, 0, 100);
		softPwmCreate(IN_1_RIGHT, 0, 100);
		softPwmCreate(IN_2_RIGHT, 0, 100);

		ROS_INFO("Motor setup done.");

	#else

		ROS_INFO("Could not set up GPIO: non-ARM device");

	#endif
}

/*
 * xIN1		xIN2	function
 * pwm		0		forward, fast decay
 * 1		pwm		forward, slow decay
 * 0		pwm		backward, fast decay
 * pwm		1		backward, slow decay
 *
 * Use the slow decay mode for now
 *
 * xIN1		xIN2	function
 * 1		pwm		forward, slow decay
 * pwm		1		backward, slow decay
 */

/* ------------------------------- left motor ------------------------------- */

void left_forward(int speed) {

	#ifdef __arm__

		int filtered_speed = filter(speed);
		softPwmWrite(IN_1_LEFT, 100);
		softPwmWrite(IN_2_LEFT, filtered_speed);

	#endif
}

void left_backward(int speed) {

	#ifdef __arm__

		int filtered_speed = filter(speed);
		softPwmWrite(IN_1_LEFT, filtered_speed);
		softPwmWrite(IN_2_LEFT, 100);

	#endif
}

void left_stop (void) {

	#ifdef __arm__

		softPwmWrite(IN_1_LEFT, 0);
		softPwmWrite(IN_2_LEFT, 0);

	#endif
}

/* ------------------------------- right motor ------------------------------ */

void right_backward(int speed) {

	#ifdef __arm__

		int filtered_speed = filter(speed);
		softPwmWrite(IN_1_RIGHT, 100);
		softPwmWrite(IN_2_RIGHT, filtered_speed);

	#endif
}

void right_forward(int speed) {

	#ifdef __arm__

		int filtered_speed = filter(speed);
		softPwmWrite(IN_1_RIGHT, filtered_speed);
		softPwmWrite(IN_2_RIGHT, 100);

	#endif
}

void right_stop (void) {

	#ifdef __arm__

		softPwmWrite(IN_1_RIGHT, 0);
		softPwmWrite(IN_2_RIGHT, 0);

	#endif
}

/* ------------------------------- both motors ------------------------------ */

void forward (int speed) {
	right_forward(speed);
	left_forward(speed);
}

void backward (int speed) {
	right_backward(speed);
	left_backward(speed);
}

void stop (void) {
	left_stop();
	right_stop();
}

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
		if (token == "stop")
			stop();
		else
			ROS_INFO("Invalid command %s", msg -> data.c_str());
	}

	// command: <direction> <speed>

	if (tokens.size() == 2) {

		// command could be "right" (right stop), "left" (left stop),
		// 		"forward" (forward <speed>), "backward" (backward <speed>)

		if (token == "right") {

			if (tokens[1] == "stop")
				right_stop();
			else
				ROS_INFO("Invalid command %s", msg -> data.c_str());

		} else if (token == "left") {

			if (tokens[1] == "stop")
				left_stop();
			else
				ROS_INFO("Invalid command %s", msg -> data.c_str());
		
		} else if (token == "forward" || token == "backward") {

			int speed = stoi(tokens[1]);
			if (token == "forward")
				forward(speed);
			else
				backward(speed);

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
				left_forward(speed);
			} else if (direction == "backward") {
				left_backward(speed);
			} else {
				ROS_INFO("Invalid command %s", msg -> data.c_str());
			}

		} else if (motor == "right") {

			if (direction == "forward") {
				right_forward(speed);
			} else if (direction == "backward") {
				right_backward(speed);
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

	hardware_setup();

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
	int update_interval = 2; // 2 seconds
	int update_interval_ms = 2000;
	ros::Rate loop_rate(0.5);

	ROS_INFO("Initialization complete");

	while (ros::ok()) {

		ros::spinOnce();

		loop_rate.sleep(); // sleep for 1 sec since we are at 1Hz
	}

	stop();

	return 0;
}
