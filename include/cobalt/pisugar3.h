#ifndef PISUGAR3_H
#define PISUGAR3_H

#include "pisugar.h"

class PiSugar3 : public PiSugar {

	private:

		/*
		 * each pisugar version has a sligtly different architecture and a slightly
		 * different way to store and retrieve informations from the registers. This
		 * procedure must be specified in the subclass
		 */

		float read_voltage (void);

		float read_current (void);

		float read_temperature (void);

};

#endif

// https://www.cplusplus.com/forum/general/107753/
