#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"

//using namespace std;

float distance[640];
int x, y;

void sensor(const sensor_msgs::LaserScan::ConstPtr& msgs)
{
	for (int i = 0; i < 640; i++) {
		distance[i] = msgs->ranges[i];
		//std::cout << distance[i] << std::endl;
	}
	//ROS_INFO("Angle [%f] and distance [%f]", msgs->angle_min, msgs->ranges[0]);
	//std::cout << "Inside the function: " << distance[7] << std::endl;
}

void humans(const std_msgs::Int32MultiArray::ConstPtr& msgh)
{
	x = msgh->data[0];
	y = msgh->data[1];
	//ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
	//std::cout << "Values are as follows: " << x << " " << y << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"asus_camera");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int32>("/turtleCommands", 10);
	ros::Publisher pub2 = nh.advertise<std_msgs::Float32MultiArray>("/humanDistance", 10);
	ros::Subscriber sub = nh.subscribe("/scan", 10, sensor);
	ros::Subscriber sub2 = nh.subscribe("/thermalHumans", 10, humans);

	std_msgs::Int32MultiArray msgh;
	sensor_msgs::LaserScan msgs;
	std_msgs::Int32 turtle;
	std_msgs::Float32MultiArray human_dis;

	ros::Rate loop_rate(10);

	//whole thermal image is in LaserScan readings between 171 and 477

	while (ros::ok())
	{
		float min = 0.0;
		human_dis.data.clear();
		//std::cout << "In the while loop before ros::ok" << std::cout;
		//if(ros::ok()){
		//std::cout << "Inside the while loop: " << distance[6] << std::endl;
		//for (int i = x+171; i < y; i + 2) {
		for(int i = 0; i < 640; i++)
		{
			if (y != 0)
			{
				if ((i >= x + 171) && (i <= y + 171))//&& (distance[i] <=2)) //if it's a range, where a human has been detected
				{
					min = distance[x + 171];
					if (distance[i] < min)
					{
						min = distance[i];
					}
					if (distance[i] <= 2)
					{
						std::cout << "A human is closer than 2 meters" << std::endl;
						turtle.data = 0;
						pub.publish(turtle); //message sent to the turtlebot to stop
					}
				}
			}
			if ((i >= 171)&&(i <= 477)) 
			{
				if (distance[i] <= 1.2) {
					turtle.data = 0;
					pub.publish(turtle);
					std::cout << "There's an obsticle closer than 1.2 meters" << std::endl;
				}
			}
			/*
			if (distance[i] <= 1) {
				std::cout << "A human is closer than 2 meters" << std::endl;
				turtle.data = 0;
				pub.publish(turtle);
				break;
			}*/
			else
			{
				turtle.data = 1;
				std::cout << "There are no obstacles within 1 m in the moving direction" << std::endl;
				pub.publish(turtle);
			}
		}
			//std::cout << i << " " << distance[i] << std::endl; 
		//std::cout << x << " " << y << std::endl;
		if (y != 0)
		{
			human_dis.data.push_back(x);
			human_dis.data.push_back(min);
			pub2.publish(human_dis);
		}
		x = 0;
		y = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}

	//ros::spin();
	return 0;
}