// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/vl53l0x/VL53L0X.hpp"

#define ToF_1 20
#define ToF_2 16
#define ToF_3 25


int main(int argc, char **argv) {

	VL53L0X sensor_1(xshutGPIOPin = ToF_1);
	VL53L0X sensor_2(xshutGPIOPin = ToF_2);
	VL53L0X sensor_3(xshutGPIOPin = ToF_3);

	VL53L0X sensor_array [3] = {sensor_1, sensor_2, sensor_3};
	int readings [3] = {0};

	try {

		for (int i = 0; i < 3; i++) {

			// initialize the sensor
			sensor_array[i].initialize();
		
			// set measurement timeout value
			sensor_array[i].setTimeout(200);
		}

	} catch (const std::exception & error) {
		std::cerr << "Error initializing sensor with reason:" << std::endl << error.what() << std::endl;
		return 1;
	}
	
	// ros node setup
	ros::init(argc, argv, "ToF_test");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>("distance", 1000);

	// 10 Hz = 100ms
	// 1000ms -> 1 Hz
	ros::Rate loop_rate(1);

	while (ros::ok()) {

		for (int i = 0; i < 3; i++) {

			try {
		
				// read the range. Note that it's a blocking call
				readings[i] = sensor.readRangeSingleMillimeters();

			} catch (const std::exception & error) {
				std::cerr << "Error geating measurement with reason:" << std::endl << error.what() << std::endl;

				i--;
			}
		}

		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		std_msgs::String msg;
		
		std::stringstream ss;
		ss << 
			"Distance[0]=" << readings[0]/10 << "cm\t" <<
			"Distance[1]=" << readings[1]/10 << "cm\t" <<
			"Distance[2]=" << readings[2]/10 << "cm";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		
		publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}


  return 0;
}