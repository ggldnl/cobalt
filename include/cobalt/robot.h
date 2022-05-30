/**
 * @file robot.cpp
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief 
 * 
 * This object encapsulates all the logic concerning the robot's mobility
 * (motors and encoders control, functions to go forward, backward
 * turn left and right, ...) and other stuff. 
 * 
 * We have two motor objects. We set a speed and they try to maintain it 
 * according to their configuration (slow decay, fast decay and so on). 
 * They have to know if the applied speed is okay or not: for example, 
 * we may have applied a speed of 80, but the engine is under load and has not 
 * enough power to get to the rpm we expect, so it should theoretically 
 * increase the speed.
 * 
 * The user does not know about all those things, he uses the library 
 * expecting that the robot will move exactly as he want to.
 * 
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"


/**
 * @brief This object encapsulates all the logic concerning the robot
 * (control of motors and encoders, functions to go forward, backward
 * and turn left and right, ...) and other stuff
 */
class Robot {

	public:

		// Motors
		Motor left_motor;
		Motor right_motor;

		Robot();

		~Robot();

		/**
		 * @brief Defines the position of the robot.
		 *
		 */
		struct Odometry {
			float x = 0.0;
			float y = 0.0;
			float theta = 0.0;
		};
		Robot::Odometry odometry;

		/**
		 * @brief initializes the hardware
		 * 
		 */
		void setup (void);

		/**
		 * @brief moves the robot forward at a certain speed
		 * 
		 */
		void forward (int speed);

		/**
		 * @brief moves the robot backward at a certain speed
		 * 
		 */
		void backward (int speed);

		/**
		 * @brief turns the robot left at a certain speed
		 * 
		 */
		void turn_left (int speed);

		/**
		 * @brief turns the robot right at a certain speed
		 * 
		 */
		void turn_right (int speed);

		/**
		 * @brief given a target position and orientation relative to the robot,
		 * reach that position
		 * 
		 * @param x 
		 * @param y 
		 * @param theta 
		 */
		void reach (float x, float y, float theta);

		/**
		 * @brief stops the robot
		 * 
		 */
		void stop (void);

	private:

		// Pin mapping

/*
		static const int IN_1_LEFT = 0;
		static const int IN_2_LEFT = 2;
		static const int ENCODER_LEFT = 23;

		static const int IN_1_RIGHT = 3;
		static const int IN_2_RIGHT = 24;
		static const int ENCODER_RIGHT = 22;

		static const int ENABLE = 7;
*/

		static const int IN_1_LEFT = 28;
		static const int IN_2_LEFT = 29;
		static const int ENCODER_LEFT = 0;

		static const int IN_1_RIGHT = 26;
		static const int IN_2_RIGHT = 27;
		static const int ENCODER_RIGHT = 7;

		static const int ENABLE = 13;

};

#endif
