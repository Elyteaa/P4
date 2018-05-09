#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include <sstream>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
//#include "beginner_tutorials/thermalRect.h"

using namespace std;
using namespace cv;

Mat frame;
vector<int> human;		
vector<int> human_prev; 
vector<Rect> temp;	    
vector<Rect> boundRect_prev; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thermal_node_cpp");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/thermalHumans", 10);

  ros::Rate loop_rate(10);

//Capturing image form the thermal camera
VideoCapture cap("http://169.254.228.255/mjpg/video.mjpg?");
//VideoCapture cap(0);
if (!cap.isOpened())
{
	cout << "Camera not found" << endl;
	getchar();
	return -1;
}

while (cap.isOpened())
{
	human_prev = human;
	human.erase(human.begin(), human.end());
		
	cap >> frame;
	cvtColor(frame, frame, CV_BGR2GRAY); 

	threshold(frame, frame, 40, 255, THRESH_BINARY);
	medianBlur(frame, frame, 5);
	morphologyEx(frame, frame, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(11, 11)));

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
		
	findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	cvtColor(frame, frame, CV_GRAY2RGB); 

	if (temp.size() > 0) {
		boundRect_prev = temp;
	}

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); 
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

	temp = boundRect;
		
	for (int i = 0; i < boundRect.size(); i++) {
		float compare = (float)boundRect[i].width / (float)boundRect[i].height;
		if ((0.2 <= compare) && (compare <= 0.7)) {
			if ((boundRect[i].width >= 45) && (boundRect[i].height >= 100)) {
				human.push_back(i);
			}
		}
	}

	for (int n = 0; n < human_prev.size(); n++) {
		for (int i = 0; i < boundRect.size(); i++) {
			float compare = (float)boundRect[i].width / (float)boundRect_prev[human_prev.at(n)].width;
			float compare2 = (float)boundRect[i].height / (float)boundRect_prev[human_prev.at(n)].height;
			if ((0.8 <= compare) && (compare <= 1.5) && (0.8 <= compare2) && (compare2 <= 1.5)) {
				human.push_back(i);
			}
		}
	}

	for (int i = 0; i < contours.size(); i++) 
	{
		Scalar color = Scalar(0, 0, 255); 
		drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
	}

	//beginner_tutorials::thermalRect msg;

	for (int j = 0; j < human.size(); j++) {
		Scalar color1 = Scalar(255, 0, 0); //blue
		putText(frame, "Human", Point(boundRect[human[j]].x, boundRect[human[j]].y), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));
		rectangle(frame, boundRect[human[j]].tl(), boundRect[human[j]].br(), color1, 2, 8, 0);

		if (ros::ok()) {
			std_msgs::Int32MultiArray msg;
			msg.data.clear();
			//std_msgs::Int32 msg2;
			int begin = boundRect[human[j]].x;
			int bend;
			bend = boundRect[human[j]].x + boundRect[human[j]].width;
			cout << begin << " " << bend << endl;
			msg.data.push_back(begin);
			msg.data.push_back(bend);
			//msg2.data = bend;
			//msg.data = [begin, end];
			chatter_pub.publish(msg);
			//chatter_pub.publish(msg2);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	imshow("sourceimg", frame);
	cvWaitKey(30);
	}
  return 0;
}
