#ifndef PISUGAR3_H
#define PISUGAR3_H

#include "pisugar.h"

/*
 * singleton: only one instance. This means that we can use a big ass constructor
 * since it's private. The singleton pattern cannot be inherited (at least not so
 * easily) so we need to adapt the same code section in each derived class
 * (I might be wrong, not an expert)
 */
class PiSugar3 : public PiSugar {
    
    
    /* -------------------------------- singleton ------------------------------- */
    
	public:
	
	    static PiSugar3* get_instance ();
    
    	PiSugar3(const PiSugar3& obj); // copy constructor
    	
    	void operator = (PiSugar3 const& obj) = delete; // assignment operator
    
    private:
    
        static PiSugar3* instance;
    
		PiSugar3();
		
    
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

		float [][] get_battery_curve (void);

	/* -------------------------- variables declaration ------------------------- */

		int fd; // file descriptor, one for each instance (?)

		float const [][] battery_curve;

	/* -------------------------------- addresses ------------------------------- */

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
