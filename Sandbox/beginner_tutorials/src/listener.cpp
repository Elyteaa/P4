#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/Twist.h>

class RobotDriver
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "/base_controller/command" topic to issue commands
	ros::Publisher cmd_vel_pub_;

public:
	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the cmd_vel topic
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	}

	//! Loop forever while sending drive commands based on keyboard input
	bool driveKeyboard()
	{		/*std::cout << "Type a command and then press enter.  "
			"Use '+' to move forward, 'l' to turn left, "
			"'r' to turn right, '.' to exit.\n";*/

		std::cout << "f + ENTER to move forward";

		//we will be sending commands of type "twist"
		geometry_msgs::Twist base_cmd;

		char cmd[50];
		while (nh_.ok()) {

			std::cin.getline(cmd, 50);
			if (cmd[0] != 'f')
			{
				std::cout << "unknown command:" << cmd << "\n";
				continue;
			}

			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			//move forward
			//if (
//cmd[0] == 'f'
//) {
				base_cmd.linear.x = 0.25;
			//}
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
			//else if (cmd[0] == '.') {
			//	break;
			//}

			//publish the assembled command
			cmd_vel_pub_.publish(base_cmd);
		}
		return true;
	}

};

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
