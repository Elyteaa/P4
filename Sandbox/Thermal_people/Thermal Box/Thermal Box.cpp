#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

using namespace std;
using namespace cv;

//Global variables
Mat frame;
vector<int> human;		//keeps indexes of blobs, which are humans
vector<int> human_prev; //contains inexes of human blobs from previous frame
vector<Rect> temp;
vector<Rect> boundRect_prev;

int main(int, char** argv)
{
	human.erase(human.begin(), human.end());
	//Mat frame = imread("C:\\Users\\eloni\\source\\repos\\ConsoleApplication17\\record.avi");
	VideoCapture cap = VideoCapture("C:\\Users\\eloni\\source\\repos\\ConsoleApplication17\\record.avi");
	//Capturing image form the thermal camera
	//VideoCapture cap("http://169.254.228.255/mjpg/video.mjpg?");
	/*if (!cap.isOpened())
	{
		cout << "Camera not found" << endl;
		getchar();
		return -1;
	}

	//while (cap.isOpened())*/
	for (int k = 0; k < cap.get(CAP_PROP_FRAME_COUNT) - 1; k++)
	{
		human.erase(human.begin(), human.end());
		//human_prev.erase(human.begin(), human.end());
		//Put the captured image in the matrix "frame"
		cap >> frame;
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

		/*if (contours.size() > 0) {
			cout << "contains: " << contours.at(0) << endl;
			cout << "size:     " << contours.size() << endl;
			waitKey(0);
		}*/

		//cout << frame.size() << endl;

		//boundRect_prev.erase(boundRect_prev.begin(), boundRect_prev.end());

		if (temp.size() >= 1) {
			cout << "Vector is filled" << endl;
			cout << temp.at(0) << endl;
			//cout << boundRect_prev.at(0) << endl;
			boundRect_prev = temp;
		}
		else { cout << "Vector empty" << endl; }

		//Giving it an input vector
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); //contours_poly[i] --> Making sure the rectangles follow our contours
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
		}

		temp.erase(temp.begin(), temp.end());
		temp = boundRect;

		//if the bounding rectangle's proportions are of human and the size is appropriate, it is recognized as a human
		for (int i = 0; i < boundRect.size(); i++) {
			float compare = (float)boundRect[i].width / (float)boundRect[i].height;
			if ((0.278 <= compare) && (compare <= 0.603)) {
				if ((boundRect[i].width >= 120) && (boundRect[i].height >= 300)) {
					human.push_back(i); //vector saves indexes of bounding boxes, which are of human ratio and acceptable height
				}
			}
		}

		//checking if any of the new blobs are a deformed previously detected human
		if (boundRect_prev.size() > 0) {
			for (int i = 0; i < boundRect.size(); i++) {
				for (int j = 0; j < boundRect_prev.size(); j++) {
					float compare = (float)boundRect[i].width / (float)boundRect_prev[i].width;
					float compare2 = (float)boundRect[i].height / (float)boundRect_prev[i].height;
					//if()
				}
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
			drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
		}

		//draw a bounding box around humans
		for (int j = 0; j < human.size(); j++) {
			Scalar color1 = Scalar(255, 0, 0); //blue
			rectangle(frame, boundRect[human[j]].tl(), boundRect[human[j]].br(), color1, 2, 8, 0);
		}

		//Show the image
		imshow("sourceimg", frame);
		//if (frame.empty()) break;
		//if (waitKey(30) >= 0) break;
	//}
		cvWaitKey(50);
	}
	return 0;
	}