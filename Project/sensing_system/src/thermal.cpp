#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <time.h> //for testing speed

using namespace std;
using namespace cv;

//Variables to save and compare detected human blobs between two frames
vector<int> human;		
vector<int> human_prev;
//Variables to save and compare bounding boxes of detected human blobs between two frames
vector<Rect> boundRect;
vector<Rect> boundRect_prev;
//Array, saving information about detected human's starting position in the image and its distance from the sensing system
float human_distance[2];

//Function saves information from the /humanDistance topic, published by the depth sensor
void callbackDist(const std_msgs::Float32MultiArray::ConstPtr& msgs)
{
	human_distance[0] = msgs->data[0];
	human_distance[1] = msgs->data[1];
}

int main(int argc, char **argv)
{
	
	clock_t start, end; //for testing
	Mat frame;
	Scalar color = Scalar(0, 0, 255); //red
	Scalar color1 = Scalar(255, 0, 0); //blue
	//stringstream name;

	ros::init(argc, argv, "thermal_node_cpp");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/humanDistance", 10, callbackDist);
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/thermalHumans", 10);

	ros::Rate loop_rate(10);

	//Capturing image form the thermal camera
	VideoCapture cap("http://169.254.229.0/mjpg/video.mjpg?");

	if (!cap.isOpened())
	{
	cout << "Camera not found" << endl;
	getchar();
	return -1;
	}

	int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("out.mp4",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

	while (cap.isOpened())
	{
		
		start = clock();
		human_prev = human;
		human.erase(human.begin(), human.end());
		
		cap >> frame;
		//Initial imae processing: thresholding, applying median blur and a morphology function - opening
		cvtColor(frame, frame, CV_BGR2GRAY); 
		threshold(frame, frame, 40, 255, THRESH_BINARY);
		medianBlur(frame, frame, 5);
		morphologyEx(frame, frame, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(11, 11)));

		//Vector to save point of the detected contours (blobs)
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
		//Converting the image back to colour, so that colourfull markers could be drawn
		cvtColor(frame, frame, CV_GRAY2RGB); 
	
		//Saving information about bounding boxes from a previous frame and preparing vector for new data
		if (boundRect.size() > 0) {
			boundRect_prev = boundRect;
			boundRect.clear();
		}

		vector<vector<Point> > contours_poly(contours.size());

		//The loop creates bounding boxes for the detected blobs and checks for human-like qualities, such as
		//appropriate bounding box ratio and minimal pixel value, as well as draws the contours
		for (int i = 0; i < contours.size(); i=hierarchy[i][0])
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); 
			boundRect.push_back(boundingRect(Mat(contours_poly[i])));
			float compare = (float)boundRect[i].width / (float)boundRect[i].height;
			if ((0.2 <= compare) && (compare <= 0.7) && (boundRect[i].width >= 45) && (boundRect[i].height >= 100)) {
				human.push_back(i);
			}
			drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
		}

		//The loop tries to find humans, identified in a previous frame, which may have gotten further/closer from
		//the sensing system
		for (int n = 0; n < human_prev.size(); n++) {
			for (int i = 0; i < boundRect.size(); i++) {
				float compare = (float)boundRect[i].width / (float)boundRect_prev[human_prev.at(n)].width;
				float compare2 = (float)boundRect[i].height / (float)boundRect_prev[human_prev.at(n)].height;
				if ((0.9 <= compare) && (compare <= 1.3) && (0.9 <= compare2) && (compare2 <= 1.3)) {
					human.push_back(i);
				}
			}
		}

		sort( human.begin(), human.end() );
		human.erase(unique(human.begin(), human.end()), human.end());
		//Send positions of detected humans to the Asus sensor, so that it can estimate distance to them
		//std::cout << human.size() << std::endl;
		for (int j = 0; j < human.size(); j++)
		{
			if (ros::ok())
			{
				std_msgs::Int32MultiArray msg;
				msg.data.clear();
				int begin = boundRect[human[j]].x;
				int bend = boundRect[human[j]].x + boundRect[human[j]].width;
				msg.data.push_back(begin);
				msg.data.push_back(bend);
				chatter_pub.publish(msg);
				loop_rate.sleep();
				ros::spinOnce();
			}
			stringstream name;
			if (human_distance[1] > 0)
			{
				name << "Human. Distance: " << human_distance[1];
				
			} else {name << "Human. Distance: N/A";}
			std::cout << "Distance, " << human_distance [1] << " " << human_distance[0] << std::endl;
			if(boundRect[human[j]].height >= 120 && boundRect[human[j]].width >= 70){
			putText(frame, name.str(), Point(human_distance[0] - 10, boundRect[human[j]].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			rectangle(frame, boundRect[human[j]].tl(), boundRect[human[j]].br(), color1, 2, 8, 0);
		}}
		imshow("Thermal blobs", frame);
		video.write(frame);
		if (frame.empty()) break;
		end = clock();
		//std::cout << "Time required for execution: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds." << std::endl;
		if (waitKey(10) >= 0) break;
		
		//ros::spinOnce();
	}
	return 0;
}
