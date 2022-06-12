// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/pisugar/pisugar.h"


int main(int argc, char **argv) {

	// get the pointer to the unique instance
	PiSugar3* pisugar_ptr = PiSugar3::get_instance();

	// variable from pointer
	PiSugar3 pisugar = *pisugar1_ptr;

	// start the background thread
	pisugar.start();

	ros::init(argc, argv, "battery_test");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>("battery", 1000);

	// 10 Hz = 100ms
	// 1000ms -> 1 Hz
	ros::Rate loop_rate(1);

	while (ros::ok()) {

		// get average voltage
		float v = pisugar.get_voltage();
		float p = pisugar.get_percent();
		float t = pisugar.get_temperature();

		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		std_msgs::String msg;
		
		std::stringstream ss;
		ss << 
			"voltage[" << v << 
			"V]\tpercentage[" << p <<  
			"%]\ttemperature[" << t << "]";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		
		publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}


  return 0;
}