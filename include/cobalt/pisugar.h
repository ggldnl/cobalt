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


    /* -------------------------------- singleton ------------------------------- */
    
    protected:
    
        PiSugar (void);

    
	/* ------------------- public methods implemented in .cpp ------------------- */

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


	/* ----------------------------- utility methods ---------------------------- */

	private:

		/**
		 * Computes the battery voltage level based on the current voltage on the provided scale
		 */
		float convert_battery_voltage_to_level (
			float battery_voltage, 
			float (*battery_curve)[2]
		);


	/* -------------------- average values computing methods -------------------- */

	protected:

		// how many entries on the voltage curve
		static const int BATTERY_CURVE_ENTRIES = 10;

	private:

		// how many voltage/current observations should we keep
		static const int HISTORY_SIZE = 30;

		// arrays to store measurements
		float voltage_measurements [HISTORY_SIZE];
		float current_measurements [HISTORY_SIZE];
		float temperature_measurements [HISTORY_SIZE];
		
		// average values
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


	/* ---------------- virtual methods implemented by subclasses --------------- */

	protected:

		/*
		 * each pisugar version has a sligtly different architecture and a slightly
		 * different way to store and retrieve informations from the registers. This
		 * procedure must be specified in the subclass
		 */

		virtual float read_voltage (void) = 0;

		virtual float read_current (void) = 0;

		virtual float read_temperature (void) = 0;

		virtual float (*(get_battery_curve)())[2] = 0;
};

#endif

// https://www.cplusplus.com/forum/general/107753/
