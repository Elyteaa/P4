#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"

int turtle_move_command;

class RobotDriver
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "/base_controller/command" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	//ros::Subscriber cmd_status_sub;

public:
	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the cmd_vel topic
		//cmd_status_sub = nh_.subscribe<std_msgs::Int32>("/turtleCommands", 10, &RobotDriver::turtleMove);
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	}

	/*void turtleMove(const std_msgs::Int32& asus_msg_turtle)
	{
		turtle_move_command = asus_msg_turtle->data;
	}*/

	//! Loop forever while sending drive commands based on keyboard input
	bool driveKeyboard()
	{
		/*std::cout << "Type a command and then press enter.  "
			"Use '+' to move forward, 'l' to turn left, "
			"'r' to turn right, '.' to exit.\n";*/

		std::cout << "f + ENTER to move forward";

		//we will be sending commands of type "twist"
		geometry_msgs::Twist base_cmd;

		char cmd[50];
		while (nh_.ok()) {

			std::cin.getline(cmd, 50);
			/*if (cmd[0] != 'f')
			{
				std::cout << "unknown command:" << cmd << "\n";
				continue;
			}*/
			
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

			if (turtle_move_command >= 1)
			{
				base_cmd.linear.x = 0.20;
			} else if (cmd[0] == 'f') {
				base_cmd.linear.x = 0.25;
			} else if (cmd[0] == 0){
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			}
			/*//turn left (yaw) and drive forward at the same time
			else if (cmd[0] == 'l') {
				base_cmd.angular.z = 0.75;
				base_cmd.linear.x = 0.25;
			}
			//turn right (yaw) and drive forward at the same time
			else if (cmd[0] == 'r') {
				base_cmd.angular.z = -0.75;
				base_cmd.linear.x = 0.25;
			}*/
			//quit
			else if (cmd[0] == '.') {
				break;
			}

			//publish the assembled command
			cmd_vel_pub_.publish(base_cmd);
		}
		return true;
	}
};

void turtleMove(const std_msgs::Int32::ConstPtr& asus_msg_turtle)
{
	turtle_move_command = asus_msg_turtle->data;
}

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotDriver driver(nh);

	ros::Subscriber cmd_status_sub = nh.subscribe("/turtleCommands", 10, turtleMove);

	driver.driveKeyboard();
}