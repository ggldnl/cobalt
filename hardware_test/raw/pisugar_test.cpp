// ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// i2c libraries
#include <wiringPiI2C.h>

// standard libraries
#include <sstream>

#define I2C_BUS 		1
#define I2C_ADDRESS 	0x57
#define I2C_CMD_VH 		0x22 // voltage high byte
#define I2C_CMD_VL		0x23 // voltage low byte
#define I2C_CMD_TEMP 	0x04

// declaring+initilizing a static 2D array in a single step
const int BATTERY_CURVE_ENTRIES = 10;
const float battery_curve[BATTERY_CURVE_ENTRIES][2] = {
	{4.10, 100.0},
	{4.05, 95.0},
	{3.90, 88.0},
	{3.80, 77.0},
	{3.70, 65.0},
	{3.62, 55.0},
	{3.58, 49.0},
	{3.49, 25.6},
	{3.32, 4.5 },
	{3.1 , 0.0 }
};

int fd; // file descriptor 

float read_voltage () {

	int vh = wiringPiI2CReadReg8(fd, I2C_CMD_VH);
	int vl = wiringPiI2CReadReg8(fd, I2C_CMD_VL);
	int v = (vh << 8) | vl;
	return v;
}

float voltage () {
	return read_voltage() / 1000.0;
}

// yes I know that passing by value is unnecessary and highly inefficient, will fix it later in the library
float convert_battery_voltage_to_level (float battery_voltage, const float battery_curve [BATTERY_CURVE_ENTRIES][2]) {

	for (int i = 0; i < BATTERY_CURVE_ENTRIES; i++) {
		float voltage_low 	= battery_curve[i][0];
		float level_low 	= battery_curve[i][1];
		if (battery_voltage > voltage_low) {
			if (i == 0) {
				return level_low;
			} else {
				float voltage_high = battery_curve[i - 1][0];
				float level_high = battery_curve[i - 1][1];
				float percent = (battery_voltage - voltage_low) / (voltage_high - voltage_low);
				return level_low + percent * (level_high - level_low);
			}
		}
	}

	return 0.0;
}

float percentage () {
	float battery_level = 100.0;
	float v = voltage();
	if (v > 0.0)
		battery_level = convert_battery_voltage_to_level(v, battery_curve);
	return battery_level;
}

float temperature () {
	return wiringPiI2CReadReg8(fd, I2C_CMD_TEMP) - 40;
}

void hardware_setup (void) {

	#ifdef __arm__

		// it returns a standard file descriptor.
		fd = wiringPiI2CSetup(I2C_ADDRESS);

		ROS_INFO("I2C setup done.");

	#else

		ROS_INFO("Could not set up GPIO: non-ARM device");

	#endif
}

int main(int argc, char **argv) {

	hardware_setup();

	ros::init(argc, argv, "battery_test");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>("battery", 1000);

	// 10 Hz = 100ms
	// 1000ms -> 1 Hz
	ros::Rate loop_rate(1);

	while (ros::ok()) {

		float v = voltage();
		int p = percentage();
		float t = temperature();

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