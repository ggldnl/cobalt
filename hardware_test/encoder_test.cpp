// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// generic includes
#include <stdint.h>  // uint8_t

// guards to allow compilation on non ARM architectures
#ifdef __arm__
#include <wiringPi.h>
#endif

#define ENCODER_PIN 0 // GPIO 0 is pin 11

/**
 * interrupt triggered count variable
 */
int count = 0;

void interrupt_handler(void) {

	count++;

	ROS_INFO("Encoder triggered [%d]", count);
};

void hardware_setup (void) {

	#ifdef __arm__

		wiringPiSetup();
		pinMode(ENCODER_PIN, INPUT);

		// Cause an interrupt when the sensor is triggered
		wiringPiISR (ENCODER_PIN, INT_EDGE_RISING, interrupt_handler);

		ROS_INFO("GPIO has been set as INPUT");

	#else

		// behavior of the node on non ARM systems

		ROS_INFO("Could not set up GPIO: non-ARM device");

	#endif

}

int  main (int  argc, char  **argv) {

	ros::init(argc, argv, "encoder_test");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1); // 1 Hz

	hardware_setup();

	while (ros::ok()) {

		// I don't know what I'm doing

		ros::spinOnce();

		loop_rate.sleep(); // sleep for 1 sec since we are at 1Hz
	}
}
