#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

int distance[640];
int x, y;

void sensor(const sensor_msgs::LaserScan::ConstPtr& msgs)
{
	for (int i = 0; i < 640; i++) {
		distance[i] = msgs->ranges[i];
	}
	//ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
}

void humans(const std_msgs::Int32MultiArray::ConstPtr& msgh)
{
	x = msgh->data[0];
	y = msgh->data[1];
	//ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"asus_camera");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int32>("/turtleCommands", 10);
	ros::Subscriber sub = nh.subscribe("/scan", 10, sensor);
	ros::Subscriber sub2 = nh.subscribe("/thermalHumans", 10, humans);

	std_msgs::Int32MultiArray msgh;
	sensor_msgs::LaserScan msgs;
	std_msgs::Int32 turtle;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		for (int i = x; i < y; i++) {
			if (distance[i] <= 2) {
				cout << "A human is closer than 2 meters" << endl;
				turtle.data = 0;
				pub.publish(turtle);
				break;
			}
			else
			{
				turtle.data = 1;
				pub.publish(turtle);
			}
		}
	}

	ros::spin();
	return 0;
}
