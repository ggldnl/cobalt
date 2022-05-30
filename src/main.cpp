/**
 * @file controller_node.cpp
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief
 * 	
 * 	ROS node
 *
 * 	subscribing to:
 * 		"/robot_command" std_msgs/String
 * 		"/robot_pid" std_msgs/String
 * 		"/robot_target_position" geometry_msgs/Pose2D
 *
 * 	publishing to:
 * 		"/robot_current_position" geometry_msgs/Pose2D
 * 		"/robot_info" ugv/Info
 * 
 * /robot_command 
 * cmd := (<direction> <speed>) | "stop" | "stop_left" | "stop_right"
 * direction := "forward" | "backward" | "left" | "right"
 * speed := integer [0, 100]
 *
 * /robot_pid
 * cmd := ("left_pid" | "right_pid" | "both") <Kp> <Ki> <Kd>
 * <Kp> <Ki> <Kd> := float
 * 
 * This way it is not mandatory to compile the same package on both the master and
 * the slave (in case they are two different machines) unless you don't want to get
 * info from the robot (ugv::Info is a custom message)
 * 
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// ros libs
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

// my libraries (the node only knows about the robot interface)
#include "../lib/robot.h"
#include "../lib/utils.h"

#include "ugv/Info.h" // robot_info

// std libraries
#include <string>
#include <vector>

/*
 * Robot interface: exports methods to control the robot
 */
Robot robot;

/*
 * ros stuff
 */
ugv::Info info_msg; // message containing general info about the robot
geometry_msgs::Pose2D pos_msg; // message containing robot position data

/*
 * only one message per time, we can't process the old ones since the robot could be 
 * in a different position with respect to when the message was sent
 */
int comm_sub_queue_size = 1;
int pos_sub_queue_size = 1;
int pid_sub_queue_size = 1;

// we can still cache the published data
int pos_pub_queue_size = 10;
int info_pub_queue_size = 10;

int update_frequency = 1; // 1Hz, 1 event each second, both for position and info

/*
 * callBack to handle commands for the robot
 * 
 * rules:
 * cmd 			:= (<direction> <speed>) | "stop" | "left_motor_stop" | "right_motor_stop"
 * direction 	:= "forward" | "backward" | "left" | "right" | 
 * 					"left_motor_forward" | "left_motor_backward" |
 * 					"right_motor_forward" | "right_motor_forward" 
 * speed 		:= integer [0, 100]
 * 
 * example:
 * $ rostopic pub -1 /robot_command std_msgs/String "forward 50"
 */
void commCallBack(const std_msgs::String::ConstPtr& msg) {

	std::vector<std::string> tokens = split(msg -> data.c_str());
	
	std::string cmd = tokens[0];
	int speed = 0;
	
	if (tokens.size() > 1)
		speed = stoi(tokens[1]);

	if (cmd == "forward") {
		robot.forward(speed);
	} else if (cmd == "backward") {
		robot.backward(speed);
	} else if (cmd == "left") {
		robot.turn_left(speed);
	} else if (cmd == "right") {
		robot.turn_right(speed);
	} else if (cmd == "left_motor_forward") {
		robot.left_motor.forward(speed);
	} else if (cmd == "right_motor_forward") {
		robot.right_motor.forward(speed);
	} else if (cmd == "left_motor_backward") {
		robot.left_motor.backward(speed);
	} else if (cmd == "right_motor_backward") {
		robot.right_motor.backward(speed);
	} else if (cmd == "left_motor_stop") {
		robot.left_motor.stop();
	} else if (cmd == "right_motor_stop") {
		robot.right_motor.stop();
	} else if (cmd == "stop") {
		robot.stop();
	}
}

/*
 * set a target position and orientation, relative to the robot (0,0 at startup)
 * Position is in cm
 *
 * example:
 * $ rostopic pub -1 /robot_target_position geometry_msgs/Pose2D 0.0 0.0 0.0
 */
void posCallBack(const geometry_msgs::Pose2D::ConstPtr& msg) {

	robot.reach(msg -> x, msg -> y, msg -> theta);
}

/* 
 * set new pid values
 * 
 * rules:
 * cmd 			:= <controller> <Kp> <Ki> <Kd>
 * controller	:= "left" | "right" | "both"
 * Kp, Ki, Kd	:= float
 * 
 * example:
 * $ rostopic pub -1 /robot_pid std_msgs/String "right 1.5 0.1 0.2"
 */
void pidCallBack(const std_msgs::String::ConstPtr& msg) {

	std::vector<std::string> tokens = split(msg -> data.c_str());
	
	std::string cmd = tokens[0];
	float p = stof(tokens[1]);
	float i = stof(tokens[2]);
	float d = stof(tokens[3]);
	
	if (cmd == "left")
		robot.left_motor.pid.set_parameters(p, i, d);
	else if (cmd == "right")
		robot.right_motor.pid.set_parameters(p, i, d);
	else if (cmd == "both") {
		robot.left_motor.pid.set_parameters(p, i, d);
		robot.right_motor.pid.set_parameters(p, i, d);
	}
}

int main (int argc, char ** argv) {

	/*
	 * Setup the hardware. The robot will enable the drv8833 and recursively call
	 * the same method on the motors that will initialize their pins and such
	 */
	robot.setup();
	robot.left_motor.pid.set_manual();
	robot.right_motor.pid.set_manual();

	/*
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it. The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 *
	 * We will have a launch file that will launches this node and the raspicam_node
	 */
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	/*
	 * subscribe <- /robot_command <- give a target direction and speed to the robot.
	 * 									It will try to keep the speed as close as possible
	 * 									to the input one
	 * 
	 * subscribe <- /robot_target_position <- give a target position and the robot will reach it
	 * 
	 * subscribe <- /robot_pid <- tune the pid values
	 *
	 * publish -> /robot_current_position -> get position and tilt angle from the robot
	 *
	 * publish -> /robot_info -> get info from the robot
	 */
	ros::Subscriber comm_sub = nh.subscribe("/robot_command", comm_sub_queue_size, commCallBack);
	ros::Subscriber pos_sub = nh.subscribe("/robot_target_position", pos_sub_queue_size, posCallBack);
	ros::Subscriber pid_sub = nh.subscribe("/robot_pid", pid_sub_queue_size, pidCallBack);

	ros::Publisher pos_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_current_position", pos_pub_queue_size);
	ros::Publisher info_pub = nh.advertise<ugv::Info>("/robot_info", info_pub_queue_size);

	/**
	 * A ros::Rate object allows you to specify a frequency that you would
	 * like to loop at. It will keep track of how long it has been since the
	 * last call to Rate::sleep(), and sleep for the correct amount of time.
	 */
	ros::Rate loop_rate(update_frequency);
	// ros::Rate loop_rate(10); // 10 Hz → 10 times a second

	while (ros::ok()) {

		/*
		 * ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by the master.
		 */
		ros::spinOnce();


		// format position message
		pos_msg.x = robot.odometry.x;
		pos_msg.y = robot.odometry.y;
		pos_msg.theta = robot.odometry.theta;

		// ROS_INFO("Sending odometry data: [x=%d, y=%d, w=%d]", odom_msg.x, odom_msg.y, odom_msg.w);
		pos_pub.publish(pos_msg);

		// format info message
		info_msg.left_current_percent = robot.left_motor.info.current_percent;
		info_msg.left_current_speed = robot.left_motor.info.current_speed;
		info_msg.left_current_rpm = robot.left_motor.info.current_rpm;
		info_msg.left_desired_percent = robot.left_motor.info.desired_percent;
		info_msg.left_desired_speed = robot.left_motor.info.desired_speed;
		info_msg.left_desired_rpm = robot.left_motor.info.desired_rpm;
		info_msg.left_encoder_count = robot.left_motor.info.encoder_count;
		info_msg.left_direction = robot.left_motor.info.direction ? "forward" : "backward";

		info_msg.right_current_percent = robot.right_motor.info.current_percent;
		info_msg.right_current_speed = robot.right_motor.info.current_speed;
		info_msg.right_current_rpm = robot.right_motor.info.current_rpm;
		info_msg.right_desired_percent = robot.right_motor.info.desired_percent;
		info_msg.right_desired_speed = robot.right_motor.info.desired_speed;
		info_msg.right_desired_rpm = robot.right_motor.info.desired_rpm;
		info_msg.right_encoder_count = robot.right_motor.info.encoder_count;
		info_msg.right_direction = robot.right_motor.info.direction ? "forward" : "backward";

		/*
		ROS_INFO("left: current_rpm=%f | current_speed=%f | current_percent=%d | desired_rpm=%f | desired_speed=%f | desired_percent=%d", 
			info_msg.left_current_rpm, info_msg.left_current_speed, info_msg.left_current_percent, 
			info_msg.left_desired_rpm, info_msg.left_desired_speed, info_msg.left_desired_percent
		);
		*/

		// ROS_INFO("left_encoder: %d\tright_encoder: %d", info_msg.left_encoder_count, info_msg.right_encoder_count);


		// ROS_INFO("Sending info data: [left_speed=%d, left_xrpm=%d, right_speed=%d, right_rpm=%d]",
		//	info_msg.left_speed, info_msg.left_rpm, info_msg.right_speed, info_msg.right_rpm);
		info_pub.publish(info_msg);

		loop_rate.sleep();
	}

	// ros is not ok anymore :(
	robot.stop();

	return 0;
}
