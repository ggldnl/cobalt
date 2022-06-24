#include "ros/ros.h"

/*
 * the wiringpi library is supposed to be already initialized
 * (calls to wiringPiSetup(), wiringPiSetupGpio() or wiringPiSetupPhys())
 */

#ifdef __arm__

#include <wiringPi.h>
#include <softPwm.h>

#endif

#include "motor.h"
#include "pid.h"

/* --------------------------- speed control stuff -------------------------- */

const float Motor::wheel_radius = 0.035; // m
const float Motor::max_rpm = 260.0;
const float Motor::max_speed = max_rpm * 0.1047 * wheel_radius; // 2 * 3.1415 / 60 = 0.1047


Motor::Motor(int ENABLE, int IN_1, int IN_2, int ENCODER):
		ENABLE(ENABLE),
        IN_1(IN_1), 
        IN_2(IN_2),
		encoder(ENCODER) {

			start(); // start the background updating thread

		}

void Motor::forward (int speed_percent) {
	desired_percent = clamp<int>(speed_percent, speed_threshold, 100);
	direction = Direction::FORWARD;
	spin_forwards(speed_percent); // try to spin at that speed
}

void Motor::backward (int speed_percent) {
	desired_percent = clamp<int>(speed_percent, speed_threshold, 100);
	direction = Direction::BACKWARD;
	spin_backwards(speed_percent); // try to spin at that speed
}

void Motor::stop () {
	desired_percent = 0;
	stop_spinning();
}

void Motor::set_slow_decay () {
	decay_mode = Decay::SLOW_DECAY;
}

void Motor::set_fast_decay () {
	decay_mode = Decay::FAST_DECAY;
}

void Motor::set_speed (int new_speed_percent) {
	desired_percent = clamp<int>(new_speed_percent, speed_threshold, 100);;
}

void Motor::setup () {

	set_interval (update_interval_ms); // background_thread -> update() function

	// pid setup
	pid.set_update_interval(update_interval_ms);
	pid.set_output_limit(0, 100); // the pid also needs to stop the motors -> 0
	pid.set_auto(); // enable the PID controller

	// hardware setup
	encoder.setup();

	#ifdef __arm__

	// enable the board
	pinMode(ENABLE, OUTPUT);
	digitalWrite (ENABLE, HIGH);
	
	softPwmCreate(IN_1, 0, 100);
	softPwmCreate(IN_2, 0, 100);

	#endif

	ROS_INFO("Motor setup done");
}

void Motor::update () {

	if (desired_percent == 0) {
		stop();
		return;
	}

	/*
	 * rotations_in_update_interval * intervals_in_second = rotations_in_second
	 * rpm = rotations_in_second * 60
	 */
	float current_rpm = (encoder.count) * (1000.0 / update_interval_ms) * 60.0;

	/*
	 * angular velocity = angular change in position of an object per second, measured 
	 * in radians / second.
	 * 
	 * Take the RPMs and divide them by 60. We get the revolutions per second.
	 * Mutliply revolutions/sec by 2 * pi to obtain the same quantity but in rad/s
	 * 
	 * 2 * 3.1415 / 60 = 0.1047
	 * 
	 * angular_velocity = rpm * 0.1047 [rad/s]
	 * 
	 * linear_speed = angular_velocity * wheel_radius [m/s]
	 * 
	 */
	float current_speed = current_rpm * 0.1047 * wheel_radius;
	current_percent = clamp<float> ((100 / current_rpm) / max_rpm, 0, 100);

	/*
	 * desired measurements
	 */

	// desired_percent set from outside

	/*
	 * desired_percent to desired_speed:
	 * 
	 * desired_percent : 100 = desired_speed : max_speed
	 * desired_speed = (desired_percent * max_speed) / 100
	 */
	float desired_speed = (desired_percent * max_speed) / 100;

	/*
	 * v = (2 * pi * r * RPM) / 60
	 *
	 * RPM = (60 * v) / (2 * pi * r)
	 */
	float desired_rpm = (60 * desired_speed) / (2 * 3.14159 * wheel_radius);

	/*
	 * pid
	 */
	float pid_output = pid.compute(desired_percent, current_percent);

	// apply it
	if (direction == Direction::FORWARD)
		spin_forwards(pid_output);
	else
		spin_backwards(pid_output);

	encoder.reset();
}

int Motor::normalize_speed (int speed) {

	/* 
	 * speed = 0 used to stop the motors
	 * but a speed value between 0 and the threshold should be ignored
	 */
	if (speed == 0)
		return 100;

	/*
	 * If we set a pwm value of 70 it means that the signal will be high for 70% of the time 
	 * and low for the remaining 30. I don't know why but this behaviour results in a 30% speed
	 * in our case (with the drv8833).
	 * To avoid this, we invert the speed argument: speed -> 100 - speed. In addition to this 
	 * the initial value should be an integer between 0 and 100, at least theoretically: from the 
	 * observations we get that with a 10% speed the wheel doesn't even move. We can set the limits 
	 * between 20 and 100 (operative limit).
	 */
	int filtered_speed = clamp<int>(speed, speed_threshold, 100);
	return 100 - filtered_speed; // (threshold, 100) speed percentage
}

void Motor::spin_forwards (int speed) {
	
	int normalized_speed = normalize_speed(speed);

	#ifdef __arm__

	if (this -> decay_mode == Decay::SLOW_DECAY) {
		softPwmWrite(IN_1, 100);
		softPwmWrite(IN_2, normalized_speed);
	} else {
		softPwmWrite(IN_1, normalized_speed);
		softPwmWrite(IN_2, 0);
	}

	#endif
}

void Motor::spin_backwards (int speed) {

	int normalized_speed = normalize_speed(speed);

	#ifdef __arm__

	if (this -> decay_mode == Decay::SLOW_DECAY) {
		softPwmWrite(IN_1, normalized_speed);
		softPwmWrite(IN_2, 100);
	} else {
		softPwmWrite(IN_1, 0);
		softPwmWrite(IN_2, normalized_speed);
	}

	#endif
}

void Motor::stop_spinning () {

	#ifdef __arm__

	softPwmWrite(IN_1, 0);
	softPwmWrite(IN_2, 0);

	#endif
}