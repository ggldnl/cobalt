// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// standard libraries
#include <sstream>

// my libraries
#include "../../include/cobalt/vl53l0x/VL53L0X.hpp"

#define ToF_1 1
#define ToF_2 25
#define ToF_3 22


int main(int argc, char **argv) {

	// number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 3;
	// GPIO pins to use for sensors' XSHUT. As exported by WiringPi.
	const uint8_t pins[SENSOR_COUNT] = { ToF_1, ToF_2, ToF_3};
	// sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6
	};

	// store the readings
	int readings [SENSOR_COUNT] = {0};

	// create sensor objects' array
	VL53L0X* sensors[SENSOR_COUNT];

	// create sensors (and ensure GPIO pin mode)
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}

	// for each sensor: create object, init the sensor (ensures power on), set timeout and address
	// note: don't power off - it will reset the address to default!
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		try {
			// initialize...
			sensors[i]->initialize();
			// ...set measurement timeout...
			sensors[i]->setTimeout(200);
			// ...set the lowest possible timing budget (high speed mode)...
			sensors[i]->setMeasurementTimingBudget(20000);
			// ...and set I2C address...
			sensors[i]->setAddress(addresses[i]);
			// ...also, notify user.
			std::cout << "Sensor " << i << " initialized, real time budget: " << sensors[i]->getMeasurementTimingBudget() << std::endl;
		} catch (const std::exception & error) {
			std::cerr << "Error initializing sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
			return 1;
		}
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
				readings[i] = sensors[i]->readRangeSingleMillimeters();

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