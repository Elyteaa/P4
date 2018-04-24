#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/tracking.hpp>


using namespace std;
using namespace cv;

//Global variables
Mat frame;
int iterations = 5;
Mat kernel;

int kernel_size;
Mat gray;
Mat binaryMat(gray.size(), gray.type());
Mat blur;

Mat edge; 
Mat draw;

Mat grayscaleMat (frame.size(), CV_8U);
Mat morph;
Mat morph1;
Mat morph2;
int thresh = 100;
int thresh_max = 255;
Mat threshMat;
int morph_size = 3;
Mat dst;
int ind = 0;
Mat binaryMat2;
int MAX_KERNEL_LENGTH = 31;


int main(int, char** argv)
{
namedWindow("video", 1);
    VideoCapture cap(0);
if(!cap.isOpened())
    {
        cout<<"Camera not found"<<endl;
        getchar();
        return -1;
    }
while ( cap.isOpened() )
    {
        cap >> frame;
	cvtColor( frame, frame, CV_BGR2GRAY );

	threshold(frame, frame, 100, 255, THRESH_BINARY_INV | THRESH_OTSU);

	imshow("Binary", frame);
	medianBlur(frame, frame, 5);
	int iterations=3;

	
	kernel_size = 3 + 2*( ind%5 );
        kernel = Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

	morphologyEx(frame, frame, MORPH_RECT, kernel, Point(-1, -1), iterations);
	imshow("morph", frame);


//Setting up vectors used to store contour points
	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
//Function used to find the contours
  findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//Vectors used to store information for the functions approxPolyDP and boundRect
	vector<vector<Point> > contours_poly(contours.size() );
	vector<Rect> boundRect(contours.size() );
//Approximating the contours and setting up the rectangle to follow the contours
  for( int i = 0; i < contours.size(); i++ )
   { 
    approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    //cout << boundRect[i].tl() << " " << boundRect[i].br() << endl;
   }
//Vector used to store the moments of the contours
   //vector<Moments> mu(contours.size() );
//Mat loca;
vector<Point>locavec;
findNonZero(frame,locavec);
//Print the values stored in locavec
/*for(vector<Point>::const_iterator i = locavec.begin(); i !=locavec.end(); i++){
cout << *i << ' ' << endl;
}*/
/*
//Defining ROI
Rect Rec(frame.rows, frame.cols, frame.rows, frame.cols);
Mat frame;	
rectangle(frame, Rec, Scalar(255),1,8,0);
Rect Rec(frame.rows/2 ,frame.cols/2, frame.rows, frame.rows);
rectangle(frame, Rec, Scalar(255),1,8,0);*/


//Function used to find non-zero pixels
/*Mat locations; 
vector<Point2i> nonzero;
findNonZero(frame==255, locations);
 for (int i = 0; i < locations.total(); i++ )
   {
        cout << "Zero#" << i << ": " << locations.at<Point>(i).x << ", " << 		locations.at<Point>(i).y << endl;
    //nonzero.assign(frame.size(), locations.at<Point>(i));
   
   }*/


//Reintroducing colour channels back into the image in order to draw contours and bounding boxes with colours
 cvtColor( frame, frame, CV_GRAY2BGR);
//Function used to calculate the moments
 // for( int i = 0; i < contours.size(); i++ )
   //  { mu[i] = moments( contours[i], false ); }
//Function used to draw contours and bounding boxes as well as print the moments
  for( int i = 0; i< contours.size(); i++ )
   {
    //printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
    Scalar color = Scalar(0,0, 255);
    Scalar color1 = Scalar(0,255, 0);
    rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color1, 2, 8, 0 );
    drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point() ); 
   }
  
  
  
//According to openCV documentation this is the fastest way to iterate over the image
  unsigned char *input = (unsigned char*)(frame.data);
  for(int j = 0;j < frame.rows;j++){
    for(int i = 0;i < frame.cols;i++){
        unsigned char b = input[frame.step * j + i*frame.channels()];
        unsigned char g = input[frame.step * j + i*frame.channels() + 1];
        unsigned char r = input[frame.step * j + i*frame.channels() + 2];
    }
}
	//Show the image
  
  imshow("sourceimg", frame);

   if(frame.empty()) break;
 	if(waitKey(30) >= 0) break;
    } 
    return 0;
}
