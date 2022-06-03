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
		// 
		// +-----+-----+---------+-Pi ZeroW-+---------+-----+-----+
		// | BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
		// +-----+-----+---------+----++----+---------+-----+-----+
		// |     |     |    3.3v |  1 || 2  | 5v      |     |     |
		// |   2 |   8 |   SDA.1 |  3 || 4  | 5v      |     |     |
		// |   3 |   9 |   SCL.1 |  5 || 6  | 0v      |     |     |
		// |   4 |   7 | GPIO. 7 |  7 || 8  | TxD     | 15  | 14  |
		// |     |     |      0v |  9 || 10 | RxD     | 16  | 15  |
		// |  17 |   0 | GPIO. 0 | 11 || 12 | GPIO. 1 | 1   | 18  |
		// |  27 |   2 | GPIO. 2 | 13 || 14 | 0v      |     |     |
		// |  22 |   3 | GPIO. 3 | 15 || 16 | GPIO. 4 | 4   | 23  |
		// |     |     |    3.3v | 17 || 18 | GPIO. 5 | 5   | 24  |
		// |  10 |  12 |    MOSI | 19 || 20 | 0v      |     |     |
		// |   9 |  13 |    MISO | 21 || 22 | GPIO. 6 | 6   | 25  |
		// |  11 |  14 |    SCLK | 23 || 24 | CE0     | 10  | 8   |
		// |     |     |      0v | 25 || 26 | CE1     | 11  | 7   |
		// |   0 |  30 |   SDA.0 | 27 || 28 | SCL.0   | 31  | 1   |
		// |   5 |  21 | GPIO.21 | 29 || 30 | 0v      |     |     |
		// |   6 |  22 | GPIO.22 | 31 || 32 | GPIO.26 | 26  | 12  |
		// |  13 |  23 | GPIO.23 | 33 || 34 | 0v      |     |     |
		// |  19 |  24 | GPIO.24 | 35 || 36 | GPIO.27 | 27  | 16  |
		// |  26 |  25 | GPIO.25 | 37 || 38 | GPIO.28 | 28  | 20  |
		// |     |     |      0v | 39 || 40 | GPIO.29 | 29  | 21  |
		// +-----+-----+---------+----++----+---------+-----+-----+
		// | BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
		// +-----+-----+---------+-Pi ZeroW-+---------+-----+-----+


		static const int IN_1_LEFT = 28;
		static const int IN_2_LEFT = 29;
		static const int ENCODER_LEFT = 0;

		static const int IN_1_RIGHT = 26;
		static const int IN_2_RIGHT = 27;
		static const int ENCODER_RIGHT = 7;

		static const int ENABLE = 13;

};

#endif
