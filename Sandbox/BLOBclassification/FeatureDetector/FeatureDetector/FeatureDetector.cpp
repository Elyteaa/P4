#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

Mat im;
Mat im_temp;

int main() {
	Mat im_or = imread("C:\\Users\\eloni\\Pictures\\thermalppl\\untitled.jpg");
	cvtColor(im_or, im, CV_RGB2GRAY);

	//setting up the detector with default prameters
	//SimpleBlobDetector detector;
	SimpleBlobDetector::Params params;

	params.filterByArea = false;
	params.minArea = 500;
	params.maxArea = 50000;
	params.filterByColor = false;
	//params.blobColor = 255;
	params.filterByInertia = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	//params.minDistBetweenBlobs = 1;

	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	//blob detection
	vector<KeyPoint> keypoints;
	//detector.detect(im, keypoints);

	// Start by creating the matrix that will allocate the new image
	//Mat img_bw(im_or.size(), im.type());

	// Apply threshhold to convert into binary image and save to new matrix
	threshold(im, im, 20, 255, THRESH_BINARY);
	medianBlur(im, im, 5);
	morphologyEx(im,im_temp, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(7,7)));
	im = im_temp;
	imshow("Processed", im);

	// Extract cordinates of blobs at their centroids, save to keypoints variable.
	detector->detect(im, keypoints);
	cout << "The size of keypoints vector is: " << keypoints.size() << endl;

	//cout << keypoints[0].pt.x << ", " << keypoints[0].pt.y << endl << keypoints[0].pt << endl;
	cout << im_or.size() << endl;

	for (int i = 0; i < keypoints.size(); i++)
	{
		//x = keypoints[i].pt[0];
		//cout << keypoints[i].pt[0] << ", " << keypoints[i].pt[1] << endl;
	}

	/*//draw detected blobs as red circles
	Mat im_with_keypoints;
	drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	*/

	Mat im_keyp;
	drawKeypoints(im_or, keypoints, im_keyp, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	imshow("keypoints", im_keyp);

	waitKey(0);
	return 0;
}