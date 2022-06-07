#include "pisugar3.h"

// for i2c communication
#include <wiringPiI2C.h>

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

	int vh = wiringPiI2CReadReg8(fd, I2C_VH);
	int vl = wiringPiI2CReadReg8(fd, I2C_VL);
	int v = (vh << 8) | vl;
	return v;
}

float PiSugar3::read_current (void) {

	int ih = wiringPiI2CReadReg8(fd, I2C_IH);
	int il = wiringPiI2CReadReg8(fd, I2C_IL);
	int i = (ih << 8) | il;
	return i;
}

float PiSugar3::read_temperature (void) {

	return wiringPiI2CReadReg8(fd, I2C_TEMP) - 40.0;
}

// need to provide for #BATTERY_CURVE_ENTRIES observations
// float** PiSugar3::get_battery_curve( ) {

	// static float** battery_curve = new float* [BATTERY_CURVE_ENTRIES];
	// for (int i = 0; i < BATTERY_CURVE_ENTRIES; ++i) {
	// 	battery_curve[i] = new float[2];
	// 	for (int j = 0; j < 2; ++j)
	// 		battery_curve[i][j] = 0.0;
	// }

// 	static const float battery_curve[BATTERY_CURVE_ENTRIES][2] = {
// 		{4.10, 100.0},
// 		{4.05, 95.0},
// 		{3.90, 88.0},
// 		{3.80, 77.0},
// 		{3.70, 65.0},
// 		{3.62, 55.0},
// 		{3.58, 49.0},
// 		{3.49, 25.6},
// 		{3.32, 4.5 },
// 		{3.1 , 0.0 }
// 	};

// 	return ptr;
// }

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

/* ------------------------------ battery curve ----------------------------- */

// declaring+initilizing a static 2D array in a single step


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