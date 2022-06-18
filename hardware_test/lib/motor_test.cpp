// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/motor/motor.h"


int main(int argc, char **argv) {

    Motor motor ();

	// ros node setup
	ros::init(argc, argv, "ToF_test");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>("distance", 1000);

	// 10 Hz = 100ms
	// 1000ms -> 1 Hz
	ros::Rate loop_rate(1);

	while (ros::ok()) {



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