
/*#include <iostream>
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
}*/

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;
Mat frame;
Mat dst;

double thresh;
double maxValue;

const int thresh_slider_max = 255;
int thresh_slider;

void on_trackbar (int, void*){
maxValue = (double) thresh_slider/thresh_slider_max;
thresh = (1.0-maxValue);
imshow ("Allahu_trackbar", dst);


}


int main(int, char** argv)
{
namedWindow("video", 1);
    VideoCapture cap("http://169.254.228.255/mjpg/video.mjpg?");
if(!cap.isOpened())
    {
        cout<<"Camera not found"<<endl;
        getchar();
        return -1;
    }
while ( cap.isOpened() )
    {
        cap >> frame;
	Mat grayscaleMat (frame.size(), CV_8U);
	cvtColor( frame, grayscaleMat, CV_BGR2GRAY );
	imshow("gray", grayscaleMat);
	threshold(grayscaleMat, dst, thresh, maxValue, THRESH_TOZERO);
	imshow("threshold", dst);
	thresh_slider = 0;
	namedWindow("Allahu_trackbar",1);
	char TrackbarName[50];
	sprintf(TrackbarName, "thresh x %d", thresh_slider_max);
	createTrackbar(TrackbarName, "Allahu_trackbar", &thresh_slider, thresh_slider_max, on_trackbar);
	on_trackbar(thresh_slider, 0);
        if(frame.empty()) break;
 	if(waitKey(30) >= 0) break;
    } 
    return 0;
}