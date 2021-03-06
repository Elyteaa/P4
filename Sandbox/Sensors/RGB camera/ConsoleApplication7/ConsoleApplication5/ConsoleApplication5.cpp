// testOpenCV.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <PvApi.h>

using namespace cv;
using namespace std;

typedef struct
{
	unsigned long   UID;
	tPvHandle       Handle;
	tPvFrame        Frame;

} tCamera;

int max_capture_width = 1936;
int max_capture_height = 1216;
int desired_width = 640;
int desired_height = 480;

//function thresholds a given greyscale image
void thresholding(int criteria, Mat greyImg) {
	for (int x = 0; x < desired_width; x++) {
		for (int y = 0; y < desired_height; y++) {
			if (greyImg.at<uchar>(y, x) <= criteria) {
				greyImg.at<uchar>(y, x) = 0;
			}
			else (greyImg.at<uchar>(y, x) = 255);
		}
	}

	/*for (int x = 0; x < max_capture_width; x++) {
		for (int y = 0; y < max_capture_height; y++) {
			greyImg.at<Vec3b>(y, x)[0] = 0;
			greyImg.at<Vec3b>(y, x)[1] = 0;
			greyImg.at<Vec3b>(y, x)[2] = 0;
		}
	}*/
}

void initialize_camera(tCamera* current_cam, int max_capture_width, int max_capture_height, int desired_width, int desired_height) {
	// Initialize the PvAPI interface so that we can look for cameras
	if (!PvInitialize()) {
		// Wait for the response from a camera after the initialization of the driver
		// This is done by checking if camera's are found yet
		while (PvCameraCount() == 0)
		{
			waitKey(15);
		}

		// If there is a camera connecte to the camera 1 interface, grab it!
		tPvCameraInfo cameraInfo;
		if (PvCameraList(&cameraInfo, 1, NULL) == 1)
		{
			unsigned long frameSize;

			// Get the camera ID
			current_cam->UID = cameraInfo.UniqueId;
			// Open the camera
			if (!PvCameraOpen(current_cam->UID, ePvAccessMaster, &(current_cam->Handle)))
			{
				// Debug
				cout << "Camera opened succesfully" << endl;

				// Get the image size of every capture
				PvAttrUint32Get(current_cam->Handle, "TotalBytesPerFrame", &frameSize);
				cout << "Framesize: " << frameSize << endl;

				// Allocate a buffer to store the image
				memset(&current_cam->Frame, 0, sizeof(tPvFrame));
				current_cam->Frame.ImageBufferSize = frameSize;
				current_cam->Frame.ImageBuffer = new char[frameSize];

				// Set maximum camera parameters - camera specific
				// Code will generate an input window from the center with the size you want
				int center_x = max_capture_width / 2;
				int center_y = max_capture_height / 2;

				// Set the manta camera parameters to get wanted frame size retrieved
				PvAttrUint32Set(current_cam->Handle, "RegionX", center_x - (desired_width / 2));
				PvAttrUint32Set(current_cam->Handle, "RegionY", center_y - (desired_height / 2));
				PvAttrUint32Set(current_cam->Handle, "Width", desired_width);
				PvAttrUint32Set(current_cam->Handle, "Height", desired_height);

				// Start the camera
				PvCaptureStart(current_cam->Handle);

				// Set the camera to capture continuously
				PvAttrEnumSet(current_cam->Handle, "AcquisitionMode", "Continuous");
				PvCommandRun(current_cam->Handle, "AcquisitionStart");
				PvAttrEnumSet(current_cam->Handle, "FrameStartTriggerMode", "Freerun");
			}
			else {
				cout << "Opening camera error" << endl;
			}
		}
		else {
			cout << "Camera not found or opened unsuccesfully" << endl;
		}
	}
	else {
		// State that we did not succeed in initializing the API
		cout << "Failed to initialise the camera API" << endl;
	}
}

int main()
{
	tCamera		myCamera;
	tPvErr		Errcode;

	// Be sure to move the windows to correct location
	// This is done to ensure that the window takes as much of the screen as possible, but can be commented out
	namedWindow("View window", 1);
	moveWindow("View window", 50, 50);

	// Initialize the camera API and perform some checks
	initialize_camera(&myCamera, max_capture_width, max_capture_height, desired_width, desired_height);

	// Create infinite loop - break out when condition is met
	// This is done for trigering the camera capture
	for (;;) {
		if (!PvCaptureQueueFrame(myCamera.Handle, &(myCamera.Frame), NULL))
		{
			while (PvCaptureWaitForFrameDone(myCamera.Handle, &(myCamera.Frame), 100) == ePvErrTimeout)
			{
			}

			////////////////////////////////////////////////////////
			// Here comes the OpenCV functionality for each frame //
			////////////////////////////////////////////////////////
			
			// Create an image header (mono image)
			// Push ImageBuffer data into the image matrix

			Mat image = Mat(myCamera.Frame.Height, myCamera.Frame.Width, CV_8U);
			image.data = (uchar *)myCamera.Frame.ImageBuffer;

			// Convert from Bayer Pattern (single channel) to BGR colour image

			Mat color =		Mat(myCamera.Frame.Height, myCamera.Frame.Width, CV_8UC3);
			Mat grey =		Mat(myCamera.Frame.Height, myCamera.Frame.Width, CV_8UC1);
			Mat edges =		Mat(myCamera.Frame.Height, myCamera.Frame.Width, CV_8UC1);

			cvtColor(image, color, CV_BayerBG2BGR);

			if (image.empty())
				cout << "empty";
			else if (image.channels()>1)
				cvtColor(image, grey, CV_BGR2GRAY);
			else grey = image;

			//thresholding(30, grey);

			imshow("Greyscale", grey);

			// Show the actual frame
			imshow("View window", color);

			Canny(grey, edges, 90, 145);
			imshow("Edges", edges);


			waitKey(20);

			// Release the image data
			image.release();
		}
	}

	// Stop the acquisition & free the camera
	Errcode = PvCommandRun(myCamera.Handle, "AcquisitionStop");
	if (Errcode != ePvErrSuccess)
		throw Errcode;

	PvCaptureEnd(myCamera.Handle);
	PvCameraClose(myCamera.Handle);

	return 0;
}
