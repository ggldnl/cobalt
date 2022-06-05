#include "pisugar.h"


PiSugar::PiSugar (void) {}

/* ----------------------------- public methods ----------------------------- */

/*
 * all these methods simply returns a variable, all the computing 
 * is done in the background loop
 */

float PiSugar::get_voltage (void) {
	return average_voltage; // TODO make atomic
}

float PiSugar::get_current (void) {
	return average_current;
}

float PiSugar::get_percent (void) {
	return average_percent;
}

float PiSugar::get_temperature (void) {
	return average_temperature;
}

void PiSugar::update (void) {

	// update the measurements
	voltage_measurements[measurement_index] = read_voltage();
	current_measurements[measurement_index] = read_current();
	temperature_measurements[measurement_index] = read_temperature();

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

// https://www.cplusplus.com/forum/general/107753/
