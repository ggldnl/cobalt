/**
 * @file motor.h
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief 
 * 
 * This module will manage a motor exporting high level methods to drive it.
 * A motor does not know if the speed at which he is running is enough or not
 * (under load) to maintain a certain rpm value: he receives an integer from 0 to 100 
 * representing the speed and uses it (PWM signal generation for the drv8833 board). 
 * That is why we need an encoder. A motor object is the model of a motor + encoder 
 * logic block.
 * 
 * Workflow:
 * - We apply a target speed from outside
 * - Encoder measure with interrupts the actual speed together with a 
 * 		thread that periodically computes the RPMs (encoder ticks in period of time)
 * - We now have a target speed (set from outside) and a current speed (measured
 * 		by encoder & thread)
 * - We can adjust the internal speed to match the desired one (PID controller)
 *
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "pid.h"

class Motor {

public:

	/**
	 * https://www.allaboutcircuits.com/technical-articles/difference-slow-decay-mode-fast-decay-mode-h-bridge-dc-motor-applications/
	 */
	static const bool SLOW_DECAY = true;
	static const bool FAST_DECAY = false;

	/*
	 * directions
	 * 
	 * if we want to stop the motor we can use either forward 0 or backward 0
	 * so it's not necessary a steady state
	 */
	static const bool FORWARD = true;
	static const bool BACKWARD = false;

	/*
	 * According to the manufacturer:
	 *
	 * Min. Operating Speed (3V): 90 +/- 10% RPM
	 * Min. Operating Speed (6V): 200 +/- 10% RPM
	 * Measured Operating Speed (6V): ~ 260 RPM avg
	 * 
	 * We are on 6v, but we can adjust this with the potentiometer up to 10v
	 */
	static const float max_rpm;
	static const float max_speed;

	/*
	 * Assert threshold is less than 50 at least -> min speed value accepted
	 */
	static const int speed_threshold = 20;

	Encoder encoder;
	PID pid;

	/**
	 * @brief Defines motor info:
	 * 
	 * desired_percent = percent value in range (threshold, 100) representing the speed.
	 * 						This is the value that we set from outside calling the
	 * 						forward(speed) and backward(speed) methods
	 * desired_rpm = desired rpm computed starting from the desired_percent
	 * desired_speed = desired speed (m/s) computed starting from the rpms
	 * 
	 * current_percent = percent value adjusted by the update() method
	 * current_rpm = current rpm value
	 * current_speed = current speed value (m/s) computed starting from the rpms
	 *
	 * encoder_counts = encoder counts (duh)
	 * 
	 * direction = direction (duh duh)
	 * 
	 */
	struct Info {

		int current_percent = 0; // duty cycle for the motors 0-100
		float current_rpm = 0.0; // current rpm
		float current_speed = 0.0; // speed [m/s]

		int desired_percent = 0;
		float desired_rpm = 0.0;
		float desired_speed = 0.0;

		int encoder_count = 0;

		bool direction = FORWARD;
	};
	Motor::Info info;

	/**
	 * @brief Constructor
	 * 
	 * @param IN_1 drv8833 input_1 pin
	 * @param IN_2 drv8833 input_2 pin
	 * @param ENCODER encoder pin
	 */
	Motor (int IN_1, int IN_2, int ENCODER);

	/*
	 * When we issue a command from outside like forward(speed) or backward(speed)
	 * the class generates the corresponding pwm and tries to drive the motor at that
	 * speed. It may not necessarily succeed, because each motor responds slightly 
	 * differently. We need a public method that sets the target speed and tries to 
	 * post it and a method, update(), that periodically tries to revise the actual
	 * speed at which the motor is spinning. 
	 * 
	 * target_speed;
	 * current_speed;
	 * 
	 * public: 
	 * 
	 * void forward (int speed) {
	 * 		target_speed = speed;
	 * 		direction = FORWARD;
	 * 		spin_forward(speed);
	 * }
	 * 
	 * void backward (int speed) {
	 * 		target_speed = speed;
	 * 		direction = BACKWARD;
	 * 		spin_backward(speed);
	 * }
	 * 
	 * void stop () {
	 * 		target_speed = 0;
	 *		// set pwm ...
	 * }
	 * 
	 * void update (void) {
	 * 		current_speed = compute_speed();
	 * 		int new_speed = pid(target_speed, current_speed);
	 * 		if(direction == FORWARD)
	 * 			spin_forward(new_speed);
	 * 		else
	 * 			spin_backward(new_speed);
	 * }
	 * 
	 * private: 
	 * 
	 * void spin_forward(int speed) {
	 * 		// set pwm ...
	 * }
	 * 
	 * void spin_backward(int speed) {
	 * 		// set pwm ...
	 * }
	 */

	/**
	 * @brief Move the motor forward at a certain speed (at least it tries to).
	 * Values out of the (threshold, 100) range will be silently ignored
	 *
	 * @param speed_percent (threshold, 100)
	 */
	void forward (int speed_percent);

	/**
	 * @brief Move the motor backward at a certain speed (at least it tries to).
	 * Values out of the (threshold, 100) range will be silently ignored
	 *
	 * @param speed_percent (threshold, 100)
	 */
	void backward (int speed_percent);

	/**
	 * @brief Stop the robot
	 * 
	 */
	void stop (void);

	/**
	 * @brief Setup the hardware (pins as output, pwm creation and so on)
	 */
	void setup (void);

	// setters

	/**
	 * @brief Choose the decay mode
	 *
	 * @param decay_mode true for slow decay, false for fast decay.
	 * Constants to choose between the two are exported by the class
	 * (SLOW_DECAY, FAST_DECAY)
	 * 
	 * e.g. set_decay_mode(Motor::FAST_DECAY);
	 */
	inline void set_decay_mode(bool decay_mode) {
		this -> _decay_mode = decay_mode;
	}

	/**
	 * @brief Set the update frequency (ms)
 	 * 
	 */
	inline void set_update_frequency (int update_interval_ms) {
		this -> update_interval_ms = update_interval_ms;
	}

	/**
	 * @brief stops the background thread that keeps updating the motors
	 * 
	 */
	inline void stop_update_thread (void) {
		this -> update_thread_running = false;
	}

private:

	// Pin mapping
	int _IN_1, _IN_2;
	
	// Internal data
	bool _decay_mode = SLOW_DECAY;
	bool _direction = FORWARD;

	bool update_thread_running = false;
	int update_interval_ms = 2000; // default value

	/**
	 * @brief Update routine: tries to correct the current speed to achieve the
	 * target speed set with forward(speed)/backward(speed);
	 */
	void update ();

	/**
	 * @brief Normalizes the input speed in range (0, 100) to fit the motors need
	 * 
	 * @param speed input in range (0, 100)
	 * @return normalized speed (0, 100) -> (100, 80);
	 */
	int normalize_speed (int speed);

	/**
	 * @brief Generates the pwm value to move the motor at the specified speed
	 * 
	 * forward:
	 * 1		pwm		forward, slow decay
	 * pwm		0		forward, fast decay
	 *
	 * @param speed percent value in range (threshold, 100)
	 */
	void spin_forwards (int speed);

	/**
	 * @brief Generates the pwm value to move the motor at the specified speed
	 * 
	 * backward:
	 * pwm		1		backward, slow decay
	 * 0		pwm		backward, fast decay
	 *
	 * @param speed percent value in range (threshold, 100)
	 */
	void spin_backwards (int speed);

	/**
	 * @brief Generates the pwm values to stop the motor
	 * 
	 */
	void stop_spinning();

};

#endif