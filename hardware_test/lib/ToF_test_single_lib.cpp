// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/vl53l0x/VL53L0X.hpp"


int main(int argc, char **argv) {

	//ToF sensor;
	VL53L0X sensor;

	try {

		// initialize the sensor
		sensor.initialize();

		// set measurement timeout value
		sensor.setTimeout(200);

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

		int distance;
		try {
			
            // read the range. Note that it's a blocking call
			distance = sensor.readRangeSingleMillimeters();

        } catch (const std::exception & error) {
			std::cerr << "Error geating measurement with reason:" << std::endl << error.what() << std::endl;
			// you may want to bail out here, depending on your application - error means issues on I2C bus read/write.
			// return 3;
			distance = 8096;
		}

		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		std_msgs::String msg;
		
		std::stringstream ss;
		ss << 
			"Distance:\t" << distance << "mm\t" << distance / 10 << "cm";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		
		publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}


  return 0;
}