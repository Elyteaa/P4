#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"
#include <unistd.h>

using namespace std;

int turtle_move_command;
unsigned int microseconds = 100;

class RobotDriver
{
private:
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_;

public:
	//ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	}

	//Loop forever; receiving commands from the /turtleCommands topic
	bool driveTurtle()
	{
		//we will be sending commands to the TurtlBot of type "twist"

		//stop the TurtleBot before the loop begins
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;	
		while (ros::ok()) {
			//if (counter > 500) {base_cmd.linear.x = 0;}
			if (turtle_move_command >= 1)
			{
				base_cmd.linear.x = 0.20;
			} else if (turtle_move_command == 0){
				base_cmd.linear.x  = 0;
			}

			//publish the assembled command and wait
			cmd_vel_pub_.publish(base_cmd);
			usleep(microseconds);

			ros::spinOnce();
		}
		return true;
	}
};

//The function saves the topic data to a global variable, which will be used inside of the class
void turtleMove(const std_msgs::Int32::ConstPtr& asus_msg_turtle)
{
	turtle_move_command = asus_msg_turtle->data;
}

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;
	ros::Subscriber cmd_status_sub = nh.subscribe("/turtleCommands", 10, turtleMove);

	RobotDriver driver(nh);
	
	//Call out the driveTurtle function from the class
	driver.driveTurtle();
}
