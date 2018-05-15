#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace cv_bridge;

/** Global variables */
String face_cascade_name, eyes_cascade_name;
CascadeClassifier face_cascade; //faces = cars
CascadeClassifier eyes_cascade; //eyes = bikes
String window_name = "Capture - Car detection";
Mat frame;
static const string OPENCV_WINDOW = "Image window";
int argc;
char** argv;
cv_bridge::CvImagePtr our_frame;

/** Function Headers */
void detectAndDisplay(Mat frame)
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(60, 60));

	for (size_t i = 0; i < faces.size(); i++)
	{
		Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
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
	}
	//-- Show what you got
	imshow(window_name, frame);
}

class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	int argc2;
	const char** argv2;

public:
	/*ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1,
			&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(OPENCV_WINDOW);
	}*/

	ImageConverter(ros::NodeHandle &nh)
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/image_raw", 1,
			&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);
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

		//ImageConverter::rgb(cv_ptr);

		/*// Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255, 0, 0));*/
	};

	void rgb() {
		ros::Rate loop_rate(10);
		while (ros::ok())
		{
			CommandLineParser parser(argc2, argv2,
				"{help h||}"
				"{face_cascade|../../data/haarcascades/cars.xml|}"
				"{eyes_cascade|../../data/haarcascades/bike.xml|}");

			parser.about("\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
				"You can use Haar or LBP features.\n\n");
			parser.printMessage();

			face_cascade_name = parser.get<String>("face_cascade");
			eyes_cascade_name = parser.get<String>("eyes_cascade");
			VideoCapture capture;

			//-- 1. Load the cascades
			if (!face_cascade.load("/home/drawn/opencv/data/haarcascades/cars.xml")) { printf("--(!)Error loading face cascade\n"); };
			if (!eyes_cascade.load("/home/drawn/opencv/data/haarcascades/bike.xml")) { printf("--(!)Error loading eyes cascade\n"); };

			//-- 2. Read the video stream //load the video
			//capture.open(0);
			//if (!capture.isOpened()) { printf("--(!)Error opening video capture\n"); return -1; }

			/*while (capture.read(frame))
			{
			if (frame.empty())
			{
			printf(" --(!) No captured frame -- Break!");
			break;
			}*/

			//-- 3. Apply the classifier to the frame
			detectAndDisplay(our_frame->image);

			if (waitKey(10) == 27) { break; } // escape

											  // Update GUI Window
			imshow(OPENCV_WINDOW, our_frame->image);

			// Output modified video stream
			image_pub_.publish(our_frame->toImageMsg());

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
};

	//void msgCallback(const sensor_msgs::Image::ConstPtr& msg);

	/** @function main */
	int main(int argc, char** argv)
	{
		ros::init(argc, argv, "RGB_node");
		ros::NodeHandle n;

		ImageConverter ic(n);
		ic.rgb();

		/*while (ros::ok())
		{
			CommandLineParser parser(argc, argv,
				"{help h||}"
				"{face_cascade|../../data/haarcascades/cars.xml|}"
				"{eyes_cascade|../../data/haarcascades/bike.xml|}");

			parser.about("\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
				"You can use Haar or LBP features.\n\n");
			parser.printMessage();

			face_cascade_name = parser.get<String>("face_cascade");
			eyes_cascade_name = parser.get<String>("eyes_cascade");
			VideoCapture capture;

			//-- 1. Load the cascades
			if (!face_cascade.load("/home/drawn/opencv/data/haarcascades/cars.xml")) { printf("--(!)Error loading face cascade\n"); return -1; };
			if (!eyes_cascade.load("/home/drawn/opencv/data/haarcascades/bike.xml")) { printf("--(!)Error loading eyes cascade\n"); return -1; };

			//-- 2. Read the video stream //load the video
			capture.open(0);
			if (!capture.isOpened()) { printf("--(!)Error opening video capture\n"); return -1; }

			while (capture.read(frame))
			{
				if (frame.empty())
				{
					printf(" --(!) No captured frame -- Break!");
					break;
				}

				//-- 3. Apply the classifier to the frame
				detectAndDisplay(frame);

				if (waitKey(10) == 27) { break; } // escape
			}
			ros::spinOnce();
			loop_rate.sleep();
		}*/
		return 0;
	}

	/*void msgCallback(const sensor_msgs::Image::ConstPtr& msg)
	{
		cout << "function entered" << endl;
		ROS_INFO("height = %d, width = %d", msg->height, msg->width);
		//frame = msg->data;
	}*/

	/** @function detectAndDisplay */
