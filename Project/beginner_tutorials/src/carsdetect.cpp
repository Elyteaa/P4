#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace cv;
using namespace cv_bridge;

String face_cascade_name, eyes_cascade_name;
CascadeClassifier face_cascade; //faces = cars
CascadeClassifier eyes_cascade; //eyes = bikes
String window_name = "Capture - Car detection";
static const string OPENCV_WINDOW = "Image window";
int argc;
char** argv;
cv_bridge::CvImagePtr our_frame;
vector<float> car_distance;
std_msgs::Int32MultiArray msg;
std::vector<Rect> faces;
Mat frame_gray;

void callbackDist(const std_msgs::Float32MultiArray::ConstPtr& msgs)
{
	car_distance.clear();
	for (int i = 0; i < msgs->data.size(); i++) {
		car_distance[i] = msgs->data[i];
	}
}

/** Function Headers */
void detect(Mat frame)
{
	msg.data.clear();

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(60, 60));

	for (int i = 0; i < faces.size(); i++)
	{
		if (ros::ok)
		{
			int begin = faces[i].x;
			int bend = faces[i].x + faces[i].width;
			msg.data.push_back(begin);
			msg.data.push_back(bend);
		}
	}
}

void display(Mat frame)
{
	int temp = 0;
	for (int i = 0; i < car_distance.size(); i + 2)
	{
		stringstream name;
		if (car_distance[1] == 0)
		{
			name << "Car. Distance: N/A";
		}
		else { name << "Car. Distance: " << car_distance[i]; };
		putText(frame, name.str(), Point(car_distance[i + 1] - 10, msg.data[i + 1] -20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
		Point center(faces[temp].x + faces[temp].width / 2, faces[temp].y + faces[temp].height / 2);
		ellipse(frame, center, Size(faces[temp].width / 2, faces[temp].height / 2), 0, 0, 360, Scalar(255, 0, 255), 3, 8, 0);

		Mat faceROI = frame_gray(faces[i]);
		std::vector<Rect> eyes;

		//-- In each face, detect eyes
		eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

		for (size_t j = 0; j < eyes.size(); j++)
		{
			Point eye_center(faces[i].x + eyes[j].x + eyes[j].width / 2, faces[i].y + eyes[j].y + eyes[j].height / 2);
			int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
			circle(frame, eye_center, radius, Scalar(255, 0, 0), 4, 8, 0);
		}
	}
	//-- Show what you got
	imshow(window_name, frame);
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1,
			&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		our_frame = cv_ptr;
		waitKey(5);

		// Output video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RGB_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber sub = n.subscribe("/carsDistance", 10, callbackDist);
	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("/rgbCars", 10);

	ImageConverter ic;

	CommandLineParser parser(argc, argv,
		"{help h||}"
		"{face_cascade|../../data/haarcascades/cars.xml|}"
		"{eyes_cascade|../../data/haarcascades/bike.xml|}");

	face_cascade_name = parser.get<String>("face_cascade");
	eyes_cascade_name = parser.get<String>("eyes_cascade");

	//Load the cascades
	if (!face_cascade.load("/home/drawn/opencv/data/haarcascades/cars.xml")) { printf("--(!)Error loading face cascade\n"); };
	if (!eyes_cascade.load("/home/drawn/opencv/data/haarcascades/bike.xml")) { printf("--(!)Error loading eyes cascade\n"); };

	while (ros::ok())
	{
		if (our_frame) {
			Mat frame = our_frame->image;
				
			//Apply the classifier to the frame
			detect(frame);

			if (msg.data.size() != 0) { pub.publish(msg); };
			loop_rate.sleep();
			ros::spinOnce();

			display(frame);

			our_frame.reset();
		}
		
	}
	return 0;
}
