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
	raspicam::RaspiCam_Cv Camera = *((raspicam::RaspiCam_Cv*) ((void**)params)[0]);
	Create* robot = (Create*)((void**)params)[1];
	cv::Mat rgb_image, bgr_image;

	while(true)
	{
		lockMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);
		robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		this_thread::sleep_for(chrono::milliseconds(500));

		Camera.grab();
		Camera.retrieve (bgr_image);
		cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		cv::imwrite("irobot_image.jpg", rgb_image);
		systemPrint(INFO_NONE, "Taking photo", THREAD_ID_IDENT_IMAGE);

		unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);

		short wallSignal = 0;

		string p1 = "./object-pictures/low-resolution/ancient-lamp-600.jpg";
		string p2 = "irobot_image.jpg";
		string p3 = "test.jpg";
		string p4 = "0.85";

		int test = ident(p1,p2,p3,p4);
		
		this_thread::sleep_for(chrono::milliseconds(1000));
	}

	return NULL;
  }