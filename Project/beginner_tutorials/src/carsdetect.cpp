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

/** Global variables */
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

void callbackDist(const std_msgs::Float32MultiArray::ConstPtr& msgs)
{
	car_distance.clear();
	for (int i = 0; i < msgs->data.size(); i++) {
		car_distance[i] = msgs->data[i];
	}
}

/** Function Headers */
void detectAndDisplay(Mat frame)
{
	msg.data.clear();
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(60, 60));

	for (size_t i = 0; i < faces.size(); i++)
	{
		int begin = faces[i].x;
		int bend;
		bend = faces[i].x + faces[i].width;
		msg.data.push_back(begin);
		msg.data.push_back(bend);
		Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
		stringstream name;
		name << "Car. Distance: " << car_distance[i + 1];
		putText(frame, name.str(), Point(car_distance[i] - 10, faces[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
		ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 3, 8, 0);

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
		ros::spinOnce();
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

	/** @function main */
	int main(int argc, char** argv)
	{
		ros::init(argc, argv, "RGB_node");
		ros::NodeHandle n;
		ros::Rate loop_rate(10);

		ros::Subscriber sub = n.subscribe("/carsDistance", 10, callbackDist);
		ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("/rgbCars", 10);

		ImageConverter ic;

		while (ros::ok())
		{
			if (our_frame) {
				Mat frame = our_frame->image;
				CommandLineParser parser(argc, argv,
					"{help h||}"
					"{face_cascade|../../data/haarcascades/cars.xml|}"
					"{eyes_cascade|../../data/haarcascades/bike.xml|}");

				parser.about("\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
					"You can use Haar or LBP features.\n\n");
				parser.printMessage();

				face_cascade_name = parser.get<String>("face_cascade");
				eyes_cascade_name = parser.get<String>("eyes_cascade");

				//-- 1. Load the cascades
				if (!face_cascade.load("/home/drawn/opencv/data/haarcascades/cars.xml")) { printf("--(!)Error loading face cascade\n"); return -1; };
				if (!eyes_cascade.load("/home/drawn/opencv/data/haarcascades/bike.xml")) { printf("--(!)Error loading eyes cascade\n"); return -1; };

				//-- 3. Apply the classifier to the frame
				detectAndDisplay(frame);
				if (msg.data.size() != 0) { pub.publish(msg); };
				our_frame.reset();
			}

			//if (waitKey(10) == 27) { break; } // escape
			ros::spinOnce();
			loop_rate.sleep();
		}
		return 0;
	}
