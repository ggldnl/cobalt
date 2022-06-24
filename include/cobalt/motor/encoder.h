#ifndef ENCODER_H
#define ENCODER_H

#include "ros/ros.h"

#include <stdint.h>

#ifdef __arm__

#include <wiringPi.h>
#include <softPwm.h>

#endif


// https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/

#define N_ISR 8 // number of objects we can create

class Encoder {

	public:

		volatile uint16_t count; // interrupt counter

		Encoder (int pin);

		~Encoder (void) {
			ISRUsed &= ~(1 << myISRId);
		}

		void reset (void) {
			count = 0;
		}

		void setup (void) {

			#ifdef __arm__
			
			pinMode(_pin, INPUT);

			#endif

			// assign ourselves a ISR ID ...
			myISRId = UINT8_MAX;
			for (uint8_t i = 0; i < N_ISR; i++) {
				if (!(ISRUsed & (1 << i))) {
					myISRId = i;
					myInstance[myISRId] = this; // record this instance
					ISRUsed |= (1 << myISRId); // lock this in the allocations table
					break;
				}
			}

			{
				static void((*ISRfunc[N_ISR])(void)) = {
				globalISR0, globalISR1, globalISR2, globalISR3,
				globalISR4, globalISR5, globalISR6, globalISR7,
				};


				#ifdef __arm__
				
				wiringPiISR(_pin, INT_EDGE_RISING, ISRfunc[myISRId]);
			
				#endif
			}

			reset();

			ROS_INFO("Encoder setup done");
		}

	private:

		/* ----------------------- software signal debouncing ----------------------- */

		static const bool debounce;
		static const long debouncing_time; // ms
		volatile unsigned long last_micros = 0l;

		/* ---------------------------- ISR bypass stuff ---------------------------- */

		int _pin;	
		uint8_t myISRId; // ISR Id for myInstance[x] and encoderISRx

		static uint8_t ISRUsed; // keep track of which ISRs are used (global bit field)
		static Encoder* myInstance[]; // callback instance for the ISR to reach instanceISR()

		void instanceISR(void) { // Instance ISR handler called from static ISR globalISRx
			if (debounce)
				debounced_interrupt();
			else
				interrupt();
			// ROS_INFO("Interrupt occurred on pin %d: %d", _pin, count);
		}

		void debounced_interrupt (void) {
			#ifdef __arm__

			if((long)(micros() - last_micros) >= debouncing_time * 1000) {
				interrupt();
				last_micros = micros();
			}

			#endif
		}

		void interrupt (void) {
			count++;
		}

		// declare all the [N_ISR] encoder ISRs
		static void globalISR0(void);
		static void globalISR1(void);
		static void globalISR2(void);
		static void globalISR3(void);
		static void globalISR4(void);
		static void globalISR5(void);
		static void globalISR6(void);
		static void globalISR7(void);
};

#endif