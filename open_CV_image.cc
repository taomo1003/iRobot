#include "robovision.hh"
#include "robotest.hh"
#include "open_CV_image.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void* open_CV_image(void* params)
{	
	raspicam::RaspiCam_Cv Camera = *((raspicam::RaspiCam_Cv*) params);
	cv::Mat rgb_image, bgr_image;

	Camera.grab();
	Camera.retrieve (bgr_image);
	cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
	cv::imwrite("irobot_image.jpg", rgb_image);
	cout << "Taking photo" << endl;

	short wallSignal = 0;

	string p1 = "./object-pictures/low-resolution/ancient-lamp-600.jpg";
	string p2 = "irobot_image.jpg";
	string p3 = "test.jpg";
	string p4 = "0.85";

	int test = ident(p1,p2,p3,p4);
	cout << "End of ident" << endl;
	return NULL;
  }