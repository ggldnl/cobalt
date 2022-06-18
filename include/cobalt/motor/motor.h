/**
 * @file motor.h
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief 
 *
 *
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

// my libraries
#include "../background_thread/background_thread.h" // pid update routine
#include "encoder.h"
#include "pid.h"

/**
 * @brief clamps the value v between the two provided thresholds
 * 
 */
template <typename T>
static T clamp(T v, T bottom, T top) {
	if (v > top)
		return top;
	if (v < bottom)
		return bottom;
	return v;
}

class Motor: public BackgroundThread {

	public:

		/**
		* https://www.allaboutcircuits.com/technical-articles/difference-slow-decay-mode-fast-decay-mode-h-bridge-dc-motor-applications/
		*/
		static const enum Decay { SLOW_DECAY, FAST_DECAY};

		/*
		* directions
		* 
		* if we want to stop the motor we can use either forward 0 or backward 0
		* so it's not necessary a steady state
		*/
		static const enum Direction { BACKWARD, FORWARD, STEADY };

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
		* Assert threshold is more than 20 at least -> min speed value accepted
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

			Direction direction = Direction.FORWARD;
		};
		Motor::Info info;

		/**
		* @brief Constructor
		* 
		* @param ENABLE drv8833 enable
		* @param IN_1 drv8833 input_1 pin
		* @param IN_2 drv8833 input_2 pin
		* @param ENCODER encoder pin
		*/
		Motor (int ENABLE, int IN_1, int IN_2, int ENCODER);

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
		* }
		* 
		* void backward (int speed) {
		* 		target_speed = speed;
		* 		direction = BACKWARD;
		* }
		* 
		* void stop () {
		* 		target_speed = 0;
		* }
		* 
		* void update (void) { // executed e.g. every 0.2 sec
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

		/*
		* The class inherits start(), stop(), pause(), resume() and update() 
		* from the BackgroundThread class. The update() method runs each 1 second
		* by default but should be increased since it specifies when the motor 
		* should update the speed (pid)
		*/

	private:

		// Pin mapping
		int _IN_1, _IN_2, _ENABLE, _ENCODER;
		
		// Internal data
		Decay _decay_mode = Decay.SLOW_DECAY;
		Direction _direction = Direction.FORWARD;

		/**
		* @brief Update routine: tries to correct the current speed to achieve the
		* target speed set with forward(speed)/backward(speed);
		*/
		void update (void);

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