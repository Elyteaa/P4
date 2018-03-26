
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	Mat img = imread("tools.jpg");
	Mat bin = Mat(img.rows, img.cols, CV_8U);
	Mat morph1 = Mat(img.rows, img.cols, CV_8U);
	Mat morph2 = Mat(img.rows, img.cols, CV_8U);
	Mat contourImg = Mat(img.rows, img.rows, CV_8UC3);

	for (int x = 0; x < img.cols; x++) {
		for (int y = 0; y < img.rows; y++) {
			if ( (img.at<Vec3b>(Point(x, y))[0] < 150 && img.at<Vec3b>(Point(x, y))[1] < 150 && img.at<Vec3b>(Point(x, y))[2] < 150)) {

				bin.at<uchar>(Point(x, y)) = 0;
			}
			else {
				bin.at<uchar>(Point(x, y)) = 255;
			}
		}

	}

	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(11, 11)); //Making the kernels for morphology
	Mat kernel2 = getStructuringElement(MORPH_RECT, Size(7, 7));
	morphologyEx(bin, morph1, MORPH_CLOSE, kernel1); //Close the binary image
	morphologyEx(morph1, morph2, MORPH_OPEN, kernel2); //Open the closed image

	//Blob detector object with default values
	//cv::SimpleBlobDetector detector;
	

	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	//detector.detect(morph2, keypoints);

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
	detector->detect( morph2, keypoints );


	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints(morph2, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Show blobs
	imshow("keypoints", im_with_keypoints);

	vector<vector<Point> > contours;

	findContours(morph2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++) {
		drawContours(contourImg, contours, i, Scalar(0, 0, 255), -1);
	}

	imshow("Input", img);
	imshow("Binary", bin);
	imshow("Closed", morph1);
	imshow("Opened", morph2);
	imshow("Contour", contourImg);
	cvWaitKey(40000);
}