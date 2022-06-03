#ifndef PISUGAR_H
#define PISUGAR_H

// for i2c communication
#include <wiringPiI2C.h>

#include "background_thread.h"

// combine both battery and RTC management
// inherits start(), stop(), pause(), resume() capabilities
// from the BackgroundThread class
class PiSugar : public BackgroundThread {

	public:	

		/**
		 * Returns the average voltage
		 */
		float get_voltage (void) {
			return average_voltage;
		}

		/**
		 * Returns the average current
		 */
		float get_current (void) {
			return average_current;
		}

		/**
		 * Returns the average battery percentage
		 */
		float get_percent (void) {
			return average_percent;
		}

		/**
		 * Returns the average battery temperature
		 */
		float get_temperature (void) {
			return average_temperature;
		}

	private:

		// how many voltage/current observations should we keep
		static const int HISTORY_SIZE = 30;

		float voltage_measurements [HISTORY_SIZE];
		float current_measurements [HISTORY_SIZE];
		float temperature_measurements [HISTORY_SIZE];
		float average_voltage = 0.0;
		float average_current = 0.0;
		float average_percent = 0.0;
		float average_temperature = 0.0;

		// index in which to put the measurement (voltage or current)
		int measurement_index = 0;
		
		// the array will fill up slowly and at first only few
		// elements will be present. Computing the mean we need to
		// account only for the values that we added
		int measurement_count = 0;

		void update (void) {

			// update the measurements
			voltage_measurements[measurement_index] = get_voltage();
			current_measurements[measurement_index] = get_current();
			temperature_measurements[measurement_index] = get_temperature();

			// update the index
			measurement_index = (measurement_index + 1) % HISTORY_SIZE;

			// update number of added elements
			if (measurement_count < HISTORY_SIZE)
				measurement_count ++; // max = HISTORY_SIZE

			// update measurements mean
			average_voltage = 0.0;
			average_current = 0.0;
			average_temperature = 0.0;
			for (int i = 0; i < measurement_count; i++) {
				// TODO possible overflows
				average_voltage += voltage_measurements[i];
				average_current += current_measurements[i];
				average_temperature += temperature_measurements[i];
			}
			average_voltage /= measurement_count;
			average_current /= measurement_count;
			average_temperature /= measurement_count;

			// update average percentage
			average_percent = 100.0;
			if (average_voltage > 0.0)
				average_percent = -1.0; // TODO implement percentage computing

		}

		float read_voltage (void);

		float read_current (void);

		float read_temperature (void);


	// 	static const int I2C_ADDR; // address

	// 	static const int I2C_CTR1; // global ctrl register 1
	// 	static const int I2C_CTR2; // global ctrl register 2

	// 	static const int I2C_TEMP; // temperature

	// 	static const int I2C_VH; // voltage high bits
	// 	static const int I2C_VL; // voltage low bits

	// 	const int I2C_IH; // current high bits
	// 	const int I2C_IL; // current low bits

	// 	const int I2C_TAP; // tap
		
	// 	const int I2C_BAT_CTRL; // battery ctrl

	// 	float battery_curve[][];

	// 	float _voltage_readings [][];
	// 	float _current_readings [][];

	// 	float _avg_voltage;
	// 	float _avg_percent;
	// 	float _avg_current;

	// 	float _read_voltage (void);

	// 	float _read_current (void);

	// 	float _read_temperature (void);

	// 	float _read_byte (int ADDR);

	// 	float _write_byte (int ADDR);

};

#endif

// https://www.cplusplus.com/forum/general/107753/
