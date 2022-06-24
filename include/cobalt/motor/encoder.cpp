#include "encoder.h"

#include "ros/ros.h"

#define N_ISR 8 // same as in the header

// interrupt handling declarations required outside the class

const bool Encoder::debounce = true;
const long Encoder::debouncing_time = 15l;

uint8_t Encoder::ISRUsed = 0; // allocation table for the globalISRx()
Encoder* Encoder::myInstance[N_ISR]; // callback instance handle for the ISR
 
// ISR for each myISRId
void Encoder::globalISR0(void) { Encoder::myInstance[0]->instanceISR(); }
void Encoder::globalISR1(void) { Encoder::myInstance[1]->instanceISR(); }
void Encoder::globalISR2(void) { Encoder::myInstance[2]->instanceISR(); }
void Encoder::globalISR3(void) { Encoder::myInstance[3]->instanceISR(); }
void Encoder::globalISR4(void) { Encoder::myInstance[4]->instanceISR(); }
void Encoder::globalISR5(void) { Encoder::myInstance[5]->instanceISR(); }
void Encoder::globalISR6(void) { Encoder::myInstance[6]->instanceISR(); }
void Encoder::globalISR7(void) { Encoder::myInstance[7]->instanceISR(); }
 
Encoder::Encoder(int pin): _pin(pin) {}