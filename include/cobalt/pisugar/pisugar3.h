#ifndef PISUGAR_3_H
#define PISUGAR_3_H


// for i2c communication
#include <wiringPiI2C.h>

#include "pisugar.h"

class PiSugar3 : public PiSugar {
	
	public:

		PiSugar3 () {
			
			// it returns a standard file descriptor.
			fd = wiringPiI2CSetup(I2C_ADDR);
		};

		~PiSugar3 () {};

	private:
		
/* ---------------- virtual methods implemented by subclasses --------------- */
		
		float read_voltage () const override {

			int vh = wiringPiI2CReadReg8(fd, I2C_VH);
			int vl = wiringPiI2CReadReg8(fd, I2C_VL);
			int v = (vh << 8) | vl;
			return v / 1000.0;
		}
		
		float read_current () const override {

			int ih = wiringPiI2CReadReg8(fd, I2C_IH);
			int il = wiringPiI2CReadReg8(fd, I2C_IL);
			int i = (ih << 8) | il;
			return i / 1000.0;
		}
		
		float read_temperature () const override {

			return wiringPiI2CReadReg8(fd, I2C_TEMP) - 40.0;
		}
		
		float (*(get_battery_curve)())[2] {
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
		
		int fd; // file descriptor


/* -------------------------------- addresses ------------------------------- */

		static int const I2C_ADDR = 0x57; // address
		
		static int const I2C_CTR1 = 0x02; // global ctrl register 1
		static int const I2C_CTR2 = 0x03; // global ctrl register 2
		
		static int const I2C_TEMP = 0x04; // temperature
		
		static int const I2C_VH = 0x22; // voltage high bits
		static int const I2C_VL = 0x23; // voltage low bits
		
		static int const I2C_IH = 0x26; // current high bits
		static int const I2C_IL = 0x27; // current low bits
		
};

class PiSugar3Creator : public PiSugarCreator {
	
	public:
		
		PiSugar* factory() const override {
			return new PiSugar3();
		}
};

#endif

/*

// get the pointer to a Creator
PiSugarCreator* creator = new PiSugar3Creator();

// get the pointer to a PiSugar3 object
PiSugar* pisugar = creator -> factory();

// start the background thread
pisugar1 -> start(); 

// do stuff
float v = pisugar -> get_voltage();
float c = pisugar -> get_current();
float p = pisugar -> get_percent();
float t = pisugar -> get_temperature();

// free the memory
delete creator;

*/
