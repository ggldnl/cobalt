#include "pisugar3.h"

// for i2c communication
#include <wiringPiI2C.h>

#include "ros/ros.h"


/* -------------------------------- singleton ------------------------------- */

PiSugar3::PiSugar3 (void) {

	// it returns a standard file descriptor.
	fd = wiringPiI2CSetup(I2C_ADDR);
}

PiSugar3* PiSugar3::instance = 0;

PiSugar3* PiSugar3::get_instance () {
	if (instance == 0)
		instance = new PiSugar3();
	return instance;
}

PiSugar3::PiSugar3(const PiSugar3& obj) {
	// do nothing
}


/* ----------------------- virtual methods definition ----------------------- */

float PiSugar3::read_voltage (void) {

	int fd = 3;

	int vh = wiringPiI2CReadReg8(fd, I2C_VH);
	int vl = wiringPiI2CReadReg8(fd, I2C_VL);
	int v = (vh << 8) | vl;
	return v / 1000.0;
}

float PiSugar3::read_current (void) {

	int fd = 3;

	int ih = wiringPiI2CReadReg8(fd, I2C_IH);
	int il = wiringPiI2CReadReg8(fd, I2C_IL);
	int i = (ih << 8) | il;
	return i / 1000.0;
}

float PiSugar3::read_temperature (void) {

	int fd = 3;

	return wiringPiI2CReadReg8(fd, I2C_TEMP) - 40.0;
}

float (*(PiSugar3::get_battery_curve)())[2] {
	static float battery_curve[BATTERY_CURVE_ENTRIES][2] = {
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
	return battery_curve;
}


/* -------------------------------- addresses ------------------------------- */

int const PiSugar3::I2C_ADDR = 0x57;
int const PiSugar3::I2C_CTR1 = 0x02;
int const PiSugar3::I2C_CTR2 = 0x03;
int const PiSugar3::I2C_TEMP = 0x04;
int const PiSugar3::I2C_VH = 0x22;
int const PiSugar3::I2C_VL = 0x23;
int const PiSugar3::I2C_IH = 0x26;
int const PiSugar3::I2C_IL = 0x27;


/* ------------------------------ usage example ----------------------------- */

/*
int main () {

	...

	// get the pointer to the unique instance
	PiSugar3* pisugar_ptr = PiSugar3::get_instance();

	// variable from pointer
	PiSugar3 pisugar = *pisugar1_ptr;

	// start the background thread
	pisugar.start();

	...

	// get average voltage
	pisugar.get_voltage();

	...
}
*/