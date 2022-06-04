#ifndef PISUGAR_H
#define PISUGAR_H

#include "background_thread.h"

/*
 * combine both battery and RTC management
 * inherits start(), stop(), pause(), resume() capabilities
 * from the BackgroundThread class. When the thread is started
 * the class begin to collect voltage, current and temperature
 * measurements over time and provide an average between them.
 * This process could be paused, resumed and stopped.
 */
class PiSugar : public BackgroundThread {

	public:	

		/**
		 * Returns the average voltage
		 */
		float get_voltage (void);

		/**
		 * Returns the average current
		 */
		float get_current (void);

		/**
		 * Returns the average battery percentage
		 */
		float get_percent (void);

		/**
		 * Returns the average battery temperature
		 */
		float get_temperature (void);

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
		
		/*
		 * the array will fill up slowly and at first only few
		 * elements will be present. Computing the mean we need to
		 * account only for the values that we added
		 */
		int measurement_count = 0;

		void update (void);

	protected: // subclasses can access it

		/*
		 * each pisugar version has a sligtly different architecture and a slightly
		 * different way to store and retrieve informations from the registers. This
		 * procedure must be specified in the subclass
		 */

		virtual float read_voltage (void);

		virtual float read_current (void);

		virtual float read_temperature (void);

		// const int I2C_ADDR; // address

		// const int I2C_CTR1; // global ctrl register 1
		// const int I2C_CTR2; // global ctrl register 2

		// const int I2C_TEMP; // temperature

		// const int I2C_VH; // voltage high bits
		// const int I2C_VL; // voltage low bits

		// const int I2C_IH; // current high bits
		// const int I2C_IL; // current low bits

		// float battery_curve[][];

};

#endif

// https://www.cplusplus.com/forum/general/107753/
