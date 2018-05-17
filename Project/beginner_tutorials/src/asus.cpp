#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"

//Array saves data from one full laser scan (58 degrees)
float distance[640];
int x, y;
std::vector<int> cars;

//The function saves laser scan data to a global variable, which will be used later
void sensor(const sensor_msgs::LaserScan::ConstPtr& msgs)
{
	for (int i = 0; i < 640; i++) {
		distance[i] = msgs->ranges[i];
	}
}

//Function saves positions of detected humans, sent by the thermal camera node
void humans(const std_msgs::Int32MultiArray::ConstPtr& msgh)
{
	x = msgh->data[0];
	y = msgh->data[1];
}

//Function saves positions of detected cars, sent by the RGB camera node
void carsCallback(const std_msgs::Int32MultiArray::ConstPtr& message)
{
	cars.clear();
	for (int i = 0; message->data.size(); i++) {
		cars[i] = message->data[i];
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"asus_camera");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int32>("/turtleCommands", 10);
	ros::Publisher pub2 = nh.advertise<std_msgs::Float32MultiArray>("/humanDistance", 10);
	ros::Publisher pub3 = nh.advertise<std_msgs::Float32MultiArray>("/carsDistance", 10);
	ros::Subscriber sub = nh.subscribe("/scan", 10, sensor);
	ros::Subscriber sub2 = nh.subscribe("/thermalHumans", 10, humans);
	ros::Subscriber sub3 = nh.subscribe("/rgbCars", 10, carsCallback);

	std_msgs::Int32 turtle;
	std_msgs::Float32MultiArray human_dis;
	std_msgs::Float32MultiArray car_dis;

	ros::Rate loop_rate(10);

	//Thermal image is in LaserScan readings between 171 and 477
	//RGB image is in all LaserScan readings, there is a margin of missing data
	while (ros::ok())
	{
		std::vector<float> min_human(1);
		bool minHumanSet = false;
		std::vector<float> min_cars(cars.size());
		bool go = true;
		std::vector<bool> minCarSet(cars.size(), false);
		human_dis.data.clear();
		car_dis.data.clear();

		//The loop checks each laser scan for appropriote attributes
		for(int i = 0; i < 640; i++)
		{
			//If there were any humans detected and the current laser scan element is within the human's position:
			if ((y != 0) && (i >= x + 171) && (i <= y + 171))
			{	
				//Setting initial minimum distance value:
				if (!minHumanSet)
				{
					min_human[0] = distance[x + 171];
					minHumanSet = true;
				}

				//Finding closest distance to the humans:
				if ((distance[i] < min_human[0]) && (distance[i] != 0)){min_human[0] = distance[i];}
				if ((distance[i] <= 2) && (distance[i] != 0))
				{
					std::cout << "A human is closer than 2 meters" << std::endl;
					//In case a human is detected closer than two meters, the TurtleBot is commanded to stop
					turtle.data = 0;
					pub.publish(turtle);
				}
			}

			//The loop calculates distance to the detected cars
			if (cars.size() != 0) {
				for (int j = 0; j < cars.size(); j + 2) {
					//If a car is in the blind spot, we skip it
					if ((cars.at(j) >= 389) && (cars.at(j) <= 1547) && (cars.at(j+1) >= 389) && (cars.at(j+1) <= 1547))
					{
						if ((!minCarSet[j]) && (distance[i] != 0)) { min_cars[j] = distance[i]; }
						if ((distance[i] <= 2.5) && (distance[i] != 0))
						{
							std::cout << "A car is closer than 2.5 meters" << std::endl;
							turtle.data = 0;
							pub.publish(turtle); //message sent to the turtlebot to stop
						}
					}
				}
			}

			//Checks for obstacles withing 28 degree angle ahead; if any detected, command is sent to stop, as a result
			//the TurtleBot will wait until the way is cleared for at least 1.2 meters ahead
			if (go && (i >= 171)&&(i <= 477)&& (distance[i] <= 1.2) && (distance[i] != 0))
			{
					go = false;
					turtle.data = 0;
					pub.publish(turtle);
				}

			//If no obstacles have been detected, the TurtleBot is commanded to go forward
			if (go)
			{
				turtle.data = 1;
				pub.publish(turtle);
			}
		}

		if (y != 0)
		{
			human_dis.data.push_back(x);
			human_dis.data.push_back(min_human[0]);
			pub2.publish(human_dis);

			//Transfer car distances back to the RGB node
			for (int i = 0; i < min_cars.size(); i + 2) {
				car_dis.data.push_back(min_cars[i]);
				car_dis.data.push_back(cars[i]);
			}
			pub3.publish(car_dis);
		}
		x = 0;
		y = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}