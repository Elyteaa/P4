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
		human_prev = human;
		human.erase(human.begin(), human.end());
		//human_prev.erase(human.begin(), human.end());
		//Put the captured image in the matrix "frame"
		cap >> frame;
		cvtColor(frame, frame, CV_BGR2GRAY); //Convert the image to grayscale

											 //Thresholding the imgae and converting it to binary & Median blur
		threshold(frame, frame, 40, 255, THRESH_BINARY);
		medianBlur(frame, frame, 5);
		morphologyEx(frame, frame, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(11, 11)));

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

		boundRect_prev.erase(boundRect_prev.begin(), boundRect_prev.end());

		if (temp.size() > 0) {
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
			if ((0.2 <= compare) && (compare <= 0.7)) {
				if ((boundRect[i].width >= 45) && (boundRect[i].height >= 100)) {
					human.push_back(i); //vector saves indexes of bounding boxes, which are of human ratio and acceptable height
				}
			}
		}

		//checking if any of the new blobs are a deformed previously detected human
		/*if (boundRect_prev.size() > 0) {
			for (int i = 0; i < boundRect.size(); i++) {
				for (int j = 0; j < boundRect_prev.size(); j++) {
					float compare = (float)boundRect[i].width / (float)boundRect_prev[j].width;
					float compare2 = (float)boundRect[i].height / (float)boundRect_prev[j].height;
					//cout << compare << " " << compare2 << endl;
					if ((0.8 <= compare) && (compare <= 1.5) && (0.8 <= compare2) && (compare2 <= 1.5)) {
						for (int n = 0; n < human_prev.size(); n++) {
							if (human_prev.at(n) == j) {
								human.push_back(i);
							}
						}
					}
				}
			}
		}*/

		for (int n = 0; n < human_prev.size(); n++) {
			for (int i = 0; i < boundRect.size(); i++) {
				float compare = (float)boundRect[i].width / (float)boundRect_prev[human_prev.at(n)].width;
				float compare2 = (float)boundRect[i].height / (float)boundRect_prev[human_prev.at(n)].height;
				if ((0.8 <= compare) && (compare <= 1.5) && (0.8 <= compare2) && (compare2 <= 1.5)) {
					human.push_back(i);
				}
			}
		}

		for (int i = 0; i < contours.size(); i++) //Loop for setting the the colour and drawing the rectangles
		{
			Scalar color = Scalar(0, 0, 255); //red
			drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
		}

		//draw a bounding box around humans
		for (int j = 0; j < human.size(); j++) {
			Scalar color1 = Scalar(255, 0, 0); //blue
			putText(frame, "Human", Point(boundRect[human[j]].x, boundRect[human[j]].y), FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 0, 255));
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