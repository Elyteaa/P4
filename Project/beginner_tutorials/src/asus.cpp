#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"

void sensor(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for (int i = 0; i < 640; i++) {
		distance[i] = msg->ranges[i];
	}
	//ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
}

void humans(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for (int i = 0; i < 640; i++) {
		distance[i] = msg->ranges[i];
	}
	//ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
}

int distance[640];

int main(int argc, char **argv){
	ros::init(argc,argv,"asus_camera");
	ros::NodeHandle nh;
	//ros::Publisher pub = nh.advertise<std_msgs::String>("/thermal", 10);
	ros::Subscriber sub = nh.subscribe("/scan", 10, sensor);
	ros::Subscriber sub2 = nh.subscribe("/thermalHumans", 10);

	std_msgs::Int32MultiArray;1

	ros::Rate loop_rate(10);



	ros::spin();
	return 0;
}