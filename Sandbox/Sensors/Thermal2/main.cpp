#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
using namespace std;
using namespace cv;


Mat frame;
Mat dst;


int main(int, char** argv)
{
    VideoCapture cap("http://169.254.229.0/mjpg/video.mjpg?");
if(!cap.isOpened())
    {
        cout<<"Camera not found"<<endl;
        getchar();
        return -1;
    }

String windowNameContrastHigh4 = "Contrast Increased by 4";
namedWindow(windowNameContrastHigh4, WINDOW_NORMAL);



while ( cap.isOpened() )
    {
        cap >> frame;
	Mat grayscaleMat (frame.size(), CV_8U);
	cvtColor( frame, grayscaleMat, CV_BGR2GRAY);

Mat contrast;
        grayscaleMat.convertTo(contrast, -1, 5, 0); //increase the contrast by 4
 imshow(windowNameContrastHigh4, contrast);

	imshow("gray", grayscaleMat);
	if(waitKey(30) >= 0) break;
    } 
    return 0;
}




