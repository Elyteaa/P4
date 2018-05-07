#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

//Global variables
Mat frame;
vector<int> human;

int main(int, char** argv)
{
	human.erase(human.begin(), human.end());
	//Capturing image form the thermal camera
	/*VideoCapture cap("http://169.254.228.255/mjpg/video.mjpg?");
	if (!cap.isOpened())
	{
		cout << "Camera not found" << endl;
		getchar();
		return -1;
	}
	while (cap.isOpened())
	{
		//Put the captured image in the matrix "frame"
		cap >> frame;*/
	Mat frame = imread("C:\\Users\\eloni\\Pictures\\thermalppl\\untitled9.jpg");
		cvtColor(frame, frame, CV_BGR2GRAY); //Convert the image to grayscale

											 //Thresholding the imgae and converting it to binary & Median blur
		threshold(frame, frame, 40, 255, THRESH_BINARY);
		medianBlur(frame, frame, 5);

		//Storing the points and details of the contours in vectors. Hierarchy = Image topology
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		//Function to find contours
		findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		cvtColor(frame, frame, CV_GRAY2RGB); //Converting the image back to RGB so we can have colourful rectangles

		cout << frame.size() << endl;

		/*for (int i = 0; i < contours.size(); i++) {
			cout << contours.at(i) << endl;
			if (i != contours.size()) { cout << "next" << endl; }
		}*/

		//Giving it an input vector
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); //contours_poly[i] --> Making sure the rectangles follow our contours
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
		}

		for (int i = 0; i < boundRect.size(); i++) {
			float compare = (float)boundRect[i].width / (float)boundRect[i].height;
			if ((0.278 <= compare) && (compare <= 0.603)) {
				human.push_back(i);
			}
		}

		/*bool isahuman = false;

		for (int i = 0; i < contours.size(); i++) {
			for (int j = 0; j < human.size(); j++) {
				if (human[j] == i) { isahuman = true; };
				break;
			}
			if (!isahuman) { boundRect.erase(boundRect.begin());
			}
			isahuman = false;
			break;
			//cout << boundRect[i].width << " " << boundRect[i].height << " " << " " << boundRect[i].x << " " << boundRect[i].y << endl;
			//if (i+1 != contours.size()) { cout << "next" << endl; }
		} //HERE*/

		for (int i = 0; i < contours.size(); i++) //Loop for setting the the colour and drawing the rectangles
		{
			Scalar color = Scalar(0, 0, 255); //red
			Scalar color1 = Scalar(255, 0, 0); //blue
			/*if (i < human.size()) {
				if (i == human[i]) {
					cout << "HUMAN" << endl;
					cout << human[i] << endl;
				}
				rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color1, 2, 8, 0);
			}*/
			for (int j = 0; j < human.size(); j++) {
				rectangle(frame, boundRect[human[j]].tl(), boundRect[i].br(), color1, 2, 8, 0);
			}
			drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
		}

		//Show the image
		imshow("sourceimg", frame);
		//if (frame.empty()) break;
		//if (waitKey(30) >= 0) break;
	//}
		waitKey(0);
	return 0;
}