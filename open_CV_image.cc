#include "robovision.hh"
#include "robotest.hh"
#include "open_CV_image.hh"
#include <stdio.h>

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

	vector<strings> imageNames;
	imageNames.push_back("ancient-lamp");
	imageNames.push_back("audio-cassette");
	imageNames.push_back("magic-lamp");
	imageNames.push_back("mammoth");
	imageNames.push_back("mayan-calendar");
	imageNames.push_back("mjolnir-hammer");
	imageNames.push_back("one-ring");	
	imageNames.push_back("pueblo-pot");
	imageNames.push_back("roman-glass");
	imageNames.push_back("willow-plate");

	for(int i = 0; i < imageNames.size(); ++i)
	{
		//if changing this low to high resolution change rename function below.
		imageNames[i] ="./object-pictures/low-resolution/" + imageNames[i] + "-600.jpg";
	}

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

		string p1 = irobot_image;
		string p2 = "test.jpg";
		string p3 = "0.85";
		for(int i = 0; i < imageNames.size();+ +i)
			{
				int test = ident(p1,imageNames[i],p2,p3);
				if (test==1) {
					imageNames.erase(i);
					int temp = rename("test.jpg",imageNames[i].substr(33))
				}
			}
		
		this_thread::sleep_for(chrono::milliseconds(1000));
	}

	return NULL;
  }