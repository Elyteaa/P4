#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdio.h>
//#include <Windows.h>
#include <iostream>
#include <time.h>
#include <ctime>

#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <vector>
#include <sstream>


using namespace cv;
using namespace std;

class BackgroundSubtractor : public Algorithm
{
public:
    virtual ~BackgroundSubtractor();
    virtual void apply(InputArray image, OutputArray fgmask, double learningRate=0);
    virtual void getBackgroundImage(OutputArray backgroundImage) const;
};


int main(int argc, const char** argv)
{

 

 
         // prepare video input
 VideoCapture cap("pedestrians2.mp4");
 
         // prepare video output
 VideoWriter outputVideo;

outputVideo.open("video4.wmv", CV_FOURCC('W', 'M', 'V', '2'), cap.get(CV_CAP_PROP_FPS), Size(640, 480), true);

        // prepare cascadeClassifier
 CascadeClassifier detectorBody;
 CascadeClassifier detectorUpper;
 // !! Put your cascade or opencv cascede into project folder !!
 string cascadeName1 = "cascadepeople.xml";
 string cascadeName2 = "cascadepeople.xml";

        // Load cascade into CascadeClassifier
 bool loaded1 = detectorBody.load(cascadeName1);
 bool loaded3 = detectorUpper.load(cascadeName2);



 

         // Basic video input loop
 for (;;)
 {

  bool Is = cap.grab();
  if (Is == false) {

   cout << "Video Capture Fail" << endl;
   break;
  }
  else {
                         // Just for measure time   
   const clock_t begin_time = clock();

                        // Store results in these 2 vectors
   vector<Rect> human;
   vector<Rect> upperBody;

                        // prepare 2 Mat container
   Mat img;
   Mat original;



   // capture frame from video file
   cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
   // Resize image if you want with same size as your VideoWriter
   resize(img, img, Size(640, 480));
                        // Store original colored image
   img.copyTo(original);

   
                        // color to gray image
   cvtColor(img, img, CV_BGR2GRAY);

                       // detect people, more remarks in performace section  
detectorBody.detectMultiScale(img, human, 1.1, 2, 0 | 1, Size(40,70), Size(80, 300));
detectorUpper.detectMultiScale(img, upperBody, 1.1, 2, 0 | 1, Size(40, 70), Size(80, 300));

   

                        // Draw results from detectorBody into original colored image
   if (human.size() > 0) {
    for (int gg = 0; gg < human.size(); gg++) {

    rectangle(original, human[gg].tl(), human[gg].br(), Scalar(0,0,255), 2, 8, 0);

    }
   }

   
                       // Draw results from detectorUpper into original colored image
   if (upperBody.size() > 0) {
    for (int gg = 0; gg < upperBody.size(); gg++) {

         rectangle(original, upperBody[gg].tl(), upperBody[gg].br(),  Scalar(255,0,0), 2, 8, 0);

    }
   }
           // measure time as current - begin_time
          clock_t diff = clock() - begin_time;
                                // convert time into string
          char buffer[126];
          printf(buffer, "%d",diff);
           // display TIME ms on original image
           putText(original, buffer, Point(100, 20), 1, 2, Scalar(255, 255, 255), 2, 8, 0);
           putText(original, "ms" , Point(150, 20), 1, 2, Scalar(255, 255, 255), 2, 8, 0);
                                // draw results
    namedWindow("prew", WINDOW_AUTOSIZE);
    
    imshow("Subtract", original);
    imshow("prew", original);
                                // make video output
    outputVideo << original; 



    int key1 = waitKey(20);

  }
 }
}
