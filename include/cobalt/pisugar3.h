#ifndef PISUGAR3_H
#define PISUGAR3_H

#include "pisugar.h"

class PiSugar3 : public PiSugar {
    
    public:
    
		PiSugar3(void);
		
    
	/* ----------------------- virtual methods declaration ---------------------- */

	private:

		/*
		 * each pisugar version has a sligtly different architecture and a slightly
		 * different way to store and retrieve informations from the registers. This
		 * procedure must be specified in the subclass
		 */

		float read_voltage (void);

		float read_current (void);

		float read_temperature (void);

	/* -------------------------- variables declaration ------------------------- */

		int fd; // file descriptor, one for each instance (?)

	protected:

		/*
		 * addresses, needs to be accessed from subclasses
		 */

		static int const I2C_ADDR;	// address

		static int const I2C_CTR1;	// global ctrl register 1
		static int const I2C_CTR2;	// global ctrl register 2

		static int const I2C_TEMP;	// temperature

		static int const I2C_VH;	// voltage high bits
		static int const I2C_VL;	// voltage low bits

		static int const I2C_IH;	// current high bits
		static int const I2C_IL;	// current low bits

};

#endif

// https://www.cplusplus.com/forum/general/107753/
