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

	/*
	 * The class inherits start(), stop(), pause(), resume() and update() 
	 * from the BackgroundThread class. The update() method runs each 1 second
	 * by default but should be increased since it specifies when the motor 
	 * should update the speed (pid).
	 */

	/* 
	 * When we issue a command from outside like forward(speed) or backward(speed)
	 * the class generates the corresponding pwm and tries to drive the motor at that
	 * speed. It may not necessarily succeed, because each motor responds slightly 
	 * differently. We need a public method that sets the target speed
	 * and a method, update(), that periodically tries to revise the actual
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

	public:

		/**
		* @brief Constructor
		* 
		* @param ENABLE drv8833 enable
		* @param IN_1 drv8833 input_1 pin
		* @param IN_2 drv8833 input_2 pin
		* @param ENCODER encoder pin
		*/
		Motor (int ENABLE, int IN_1, int IN_2, int ENCODER);
 
		/**
		 * @brief Move the motor forward at a certain speed (at least it tries to).
		 * Values out of the (threshold, 100) range will be clamped to the range bounds
		 *
		 * @param speed_percent (threshold, 100)
		 */
		void forward (int speed);

		/**
		* @brief Move the motor backward at a certain speed (at least it tries to).
		* Values out of the (threshold, 100) range will be clamped to the range bounds
		*
		* @param speed_percent (threshold, 100)
		*/
		void backward (int speed);

		/**
		 * @brief Change the motor speed keeping the same direction
		 */
		void set_speed (int new_speed_percent);

		/**
		* @brief Stop the motor
		*/
		void stop (void);

		/**
		* @brief Setup the hardware (pins as output, pwm creation and so on)
		*/
		void setup (void);

		/* --------------------------------- setters -------------------------------- */

		/**
		 * @brief set slow decay mode
		 */
		void set_slow_decay();

		/**
		 * @brief set fast decay mode
		 */
		void set_fast_decay();

	private:

		/* ------------------------------- pin mapping ------------------------------ */

		int IN_1, IN_2, ENABLE;

		/* ------------------------------ internal data ----------------------------- */

		/**
		 * https://www.allaboutcircuits.com/technical-articles/difference-slow-decay-mode-fast-decay-mode-h-bridge-dc-motor-applications/
		 */
		enum Decay { SLOW_DECAY, FAST_DECAY};

		/*
		 * current direction in which the motor is spinning
		 */
		enum Direction { BACKWARD, FORWARD};

		Encoder encoder;
		PID pid;
		
		Decay decay_mode = Decay::SLOW_DECAY;
		Direction direction = Direction::FORWARD;

		/*
		 * assert threshold is more than 20 at least -> min speed value accepted
		 */
		static const int speed_threshold = 20;
		
		int update_interval_ms = 500;

		/*
		 * According to the manufacturer:
		 *
		 * Min. Operating Speed (3V): 90 +/- 10% RPM
		 * Min. Operating Speed (6V): 200 +/- 10% RPM
		 * Measured Operating Speed (6V): ~ 260 RPM avg
		 * 
		 * We are on 6v, but we can adjust this with the potentiometer up to 10v
		 */
		 // TODO update measurements from motor datasheet
		static const float wheel_radius;
		static const float max_rpm;
		static const float max_speed;

		/* 
		 * desired_percent -> set from outside
		 * current_percent -> percent over max speed (physically) reachable by the motor
		 */
		float desired_percent;
		float current_percent;

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
		*/
		void stop_spinning();

};

#endif