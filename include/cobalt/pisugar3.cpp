#include "pisugar3.h"

// for i2c communication
#include <wiringPiI2C.h>


PiSugar3::PiSugar3 (void) {

	// it returns a standard file descriptor.
	fd = wiringPiI2CSetup(I2C_ADDR);
}

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

/*
 * constants
 */
int const PiSugar3::I2C_ADDR = 0x57;
int const PiSugar3::I2C_CTR1 = 0x02;
int const PiSugar3::I2C_CTR2 = 0x03;
int const PiSugar3::I2C_TEMP = 0x04;
int const PiSugar3::I2C_VH = 0x22;
int const PiSugar3::I2C_VL = 0x23;
int const PiSugar3::I2C_IH = 0x26;
int const PiSugar3::I2C_IL = 0x27;
