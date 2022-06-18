#include "ros/ros.h"

/*
 * the library is supposed to be already initialized
 * (calls to wiringPiSetup(), wiringPiSetupGpio() or wiringPiSetupPhys())
 */

#ifdef __arm__

#include <wiringPi.h>
#include <softPwm.h>

#endif

#include <thread>

#include "geometry.h"
#include "motor.h"
#include "pid.h"
#include "utils.h"

const float Motor::max_rpm = 260.0;
const float Motor::max_speed = max_rpm * 0.1047 * robot_geometry.wheel_radius; // 2 * 3.1415 / 60 = 0.1047

Motor::Motor(int IN_1, int IN_2, int ENCODER):
		_IN_1(IN_1), _IN_2(IN_2),
		encoder(ENCODER) {}

void Motor:: setup (void) {

	// pid setup
	pid.set_update_interval(update_interval_ms);
	pid.set_output_limit(0, 100); // the pid also needs to stop the motors -> 0
	pid.set_auto();

	// hardware setup
	encoder.setup();

	#ifdef __arm__

	softPwmCreate(_IN_1, 0, 100);
	softPwmCreate(_IN_2, 0, 100);

	#endif

	/*
	 * start a routine to constantly update the motor
	 */
	update_thread_running = false; // TODO make variable atomic
	std::thread( // lambda 
		// https://thispointer.com/c11-lambda-how-to-capture-member-variables-inside-lambda-function/
		[this]() { // [list of captured variables] (parameters)
			while (this -> update_thread_running) {
				this -> update();
				std::this_thread::sleep_for(
					std::chrono::milliseconds(this -> update_interval_ms)
				);
			}
		}).detach();
	ROS_INFO("Update thread routine running");

	ROS_INFO("Motor setup done");
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

/*
 * xIN1		xIN2	function
 * pwm		0		forward, fast decay
 * 1		pwm		forward, slow decay
 * 0		pwm		backward, fast decay
 * pwm		1		backward, slow decay
 *
 * slow decay:
 * 1		pwm		forward, slow decay
 * pwm		1		backward, slow decay
 *
 * fast decay:
 * pwm		0		forward, fast decay
 * 0		pwm		backward, fast decay
 *
 * forward:
 * 1		pwm		forward, slow decay
 * pwm		0		forward, fast decay
 *
 * backward:
 * pwm		1		backward, slow decay
 * 0		pwm		backward, fast decay
 * 
 * pwm value in range (0, 100) (wiringPi)
 */

/*
 * These methods do NOT set the target speed, so it is safe to call them 
 * inside the update routine. They only compute and apply the pwm
 */

void Motor::spin_forwards (int speed) {
	int normalized_speed = normalize_speed(speed);

	#ifdef __arm__

	if (this -> _decay_mode == SLOW_DECAY) {
		softPwmWrite(_IN_1, 100);
		softPwmWrite(_IN_2, normalized_speed);
	} else {
		softPwmWrite(_IN_1, normalized_speed);
		softPwmWrite(_IN_2, 0);
	}

	#endif
}

void Motor::spin_backwards (int speed) {
	int normalized_speed = normalize_speed(speed);

	#ifdef __arm__

	if (this -> _decay_mode == SLOW_DECAY) {
		softPwmWrite(_IN_1, normalized_speed);
		softPwmWrite(_IN_2, 100);
	} else {
		softPwmWrite(_IN_1, 0);
		softPwmWrite(_IN_2, normalized_speed);
	}

	#endif
}

void Motor::stop_spinning() {

	#ifdef __arm__

	softPwmWrite(_IN_1, 0);
	softPwmWrite(_IN_2, 0);

	#endif	
}

/*
 * these methos sets the target speed and the direction. 
 * Use them from the outside of the class
 */

void Motor::forward(int speed) {
	info.desired_percent = clamp<int>(speed, speed_threshold, 100);;
	info.direction = FORWARD;
	spin_forwards(speed); // try to spin at that speed
}

void Motor::backward(int speed) {
	info.desired_percent = clamp<int>(speed, speed_threshold, 100);;
	info.direction = BACKWARD;
	spin_backwards(speed); // try to spin at that speed
}

void Motor::stop(void) {
	info.desired_percent = 0;
	stop_spinning();
}

void Motor::update () {

	/*
	 * current measurements
	 */

	info.encoder_count = encoder.count;
	
	/*
	 * rotations_in_update_interval * intervals_in_second = rotations_in_second
	 * rpm = rotations_in_second * 60
	 */
	info.current_rpm = (encoder.count / encoder.disk_slots) * (1000.0 / update_interval_ms) * 60.0;

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
	info.current_speed = info.current_rpm * 0.1047 * robot_geometry.wheel_radius;

	info.current_percent = clamp<float> ((100 / info.current_rpm) / max_rpm, 0, 100);

	/*
	 * desired measurements
	 */

	// info.desired_percent set from outside

	/*
	 * desired_percent to desired_speed:
	 * 
	 * desired_percent : 100 = desired_speed : max_speed
	 * desired_speed = (desired_percent * max_speed) / 100
	 */
	info.desired_speed = (info.desired_percent * max_speed) / 100;

	/*
	 * v = (2 * pi * r * RPM) / 60
	 *
	 * RPM = (60 * v) / (2 * pi * r)
	 */
	info.desired_rpm = (60 * info.desired_speed) / (2 * 3.14159 * robot_geometry.wheel_radius);

	/*
	 * pid
	 */

	float pid_output = pid.compute(info.desired_percent, info.current_percent);

	// apply it
	if (info.direction == FORWARD)
		spin_forwards(pid_output);
	else
		spin_backwards(pid_output);

	encoder.reset();

}
