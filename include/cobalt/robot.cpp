#include "ros/ros.h"

#include "robot.h"

#ifdef __arm__
#include  <wiringPi.h>
#endif

Robot::Robot () :
	left_motor(IN_1_LEFT, IN_2_LEFT, ENCODER_LEFT),
	right_motor(IN_1_RIGHT, IN_2_RIGHT, ENCODER_RIGHT) {
	}

Robot::~Robot() {}

void Robot::forward (int speed) {
	left_motor.forward(speed);
	right_motor.forward(speed);
}

void Robot::backward (int speed) {
	left_motor.backward(speed);
	right_motor.backward(speed);
}

void Robot::turn_left (int speed) {
	left_motor.backward(speed);
	right_motor.forward(speed);
}

void Robot::turn_right (int speed) {
	left_motor.forward(speed);
	right_motor.backward(speed);
}

void Robot::reach(float x, float y, float theta) {

	// how many turns the left and right wheel need to do?

}

void Robot::stop (void) {

	// stop the threads
	left_motor.stop_update_thread();
	right_motor.stop_update_thread();

	// stop the motor if it is spinning
	left_motor.stop();
	right_motor.stop();
}

void Robot::setup(void) {

	#ifdef __arm__
	
	/*
	 * WiringPi setup. You must initialise WiringPi with one of wiringPiSetup(),
	 * wiringPiSetupGpio() or wiringPiSetupPhys() functions.
	 * wiringPiSetupSys() is not fast enough, so you must run your programs with sudo.
	 *
	 * We will use the library convention
	 */
	wiringPiSetup();

	/*
	 * Enable the drv8833 board. The rest of the configuration
	 * is delegated to the motors
	 */
	pinMode(ENABLE, OUTPUT);
	digitalWrite(ENABLE, HIGH);
	
	#endif

	/*
	 * Setup the motors
	 */
	left_motor.setup();
	right_motor.setup();

	ROS_INFO("Setup done");	
}