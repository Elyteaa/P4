// thermalcamv3.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv/highgui.h>

using namespace cv;

int main(int argc, char *argv[])
{
	Mat frame;
	namedWindow("video", 1);
	VideoCapture cap("http://169.254.228.255/mjpg/video.mjpg?");
	while (cap.isOpened())
	{
		cap >> frame;
		if (frame.empty()) break;

		imshow("video", frame);
		if (waitKey(30) >= 0) break;
	}

	return 0;
}