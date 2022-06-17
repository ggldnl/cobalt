#ifndef PISUGAR_1_H
#define PISUGAR_1_H

#include "pisugar.h"

/**
 * Concrete Products provide various implementations of the Product interface
 */
class PiSugar1 : public PiSugar {
	
	public:

		PiSugar1 () {
			
			// it returns a standard file descriptor.
			fd = wiringPiI2CSetup(I2C_ADDR);
		};

		~PiSugar1 () {};
		
		

	private:

		float read_voltage () const override {
			return 0.0;
		}
		
		float read_current () const override {
			return 0.0;
		}
		
		float read_temperature () const override {
			return 0.0;
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

/**
 * Concrete Creators override the factory method in order to change the
 * resulting product's type
 */
class PiSugar1Creator : public PiSugarCreator {

  /**
   * Note that the signature of the method still uses the abstract product type,
   * even though the concrete product is actually returned from the method. This
   * way the Creator can stay independent of concrete product classes
   */
	public:

		PiSugar* factory() const override {
			return new PiSugar1();
		}
};

#endif