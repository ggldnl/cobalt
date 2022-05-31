/**
 * @file pisugar.h
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief 
 * 
 * Interface for all pisugar models. 
 *
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PISUGAR_H
#define PISUGAR_H

// i2c library
#include <wiringPiI2C.h>

// combine both battery and RTC management
class PiSugar {

	public:

		/**
		 * Returns the voltage. Instant measure, not accurate
		 */
		float get_voltage ();

		/**
		 * Returns the current. Instant measure, not accurate
		 */
		float get_current ();

		/**
		 * Returns the estimated battery percentage. 
		 * Instant measure, not accurate
		 */
		float get_percent ();


		float get_temperature ();	

		/**
		 * Returns the average voltage
		 */
		float get_avg_voltage ();

		float get_avg_current ();

		float get_avg_percent ();

	private:


		const int int I2C_ADDR; // address

		const int I2C_CTR1; // global ctrl register 1
		const int I2C_CTR2; // global ctrl register 2

		const int I2C_TEMP; // temperature

		const int I2C_VH; // voltage high bits
		const int I2C_VL; // voltage low bits

		const int I2C_IH; // current high bits
		const int I2C_IL; // current low bits

		const int I2C_TAP; // tap
		
		const int I2C_BAT_CTRL; // battery ctrl

		float battery_curve[][];

		float _voltage_readings [][];
		float _current_readings [][];

		float _avg_voltage;
		float _avg_percent;
		float _avg_current;

		float _read_voltage (void);

		float _read_current (void);

		float _read_temperature (void);

		float _read_byte (int ADDR);

		float _write_byte (int ADDR);

}

#endif

// https://www.cplusplus.com/forum/general/107753/
