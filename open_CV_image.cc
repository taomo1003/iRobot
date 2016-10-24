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

	vector<string> imageNames;

	string p1 = "irobot_image.jpg";
	string p2 = "test.jpg";
	string p3 = "0.90";

	string p4[N_IMAGE_THREADS];

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

	vector<pthread_t> imageThread(N_IMAGE_THREADS);
	vector<void**> thdParams(N_IMAGE_THREADS);

	for(int i = 0; i < imageNames.size(); ++i)
	{
		//if changing this low to high resolution change rename function below.
		imageNames[i] =(string)"./object-pictures/low-resolution/" + imageNames[i] + (string)"-600.jpg";
	}

	for (int i = 0; i < N_IMAGE_THREADS; ++i)
	{
		void** params = new void*[4];
		params[0] = &imageNames[i];
		params[1] = &p1;
		p4[i] = ((string) "test" + to_string(i) + (string)".jpg");
		params[2] = &p4[i];
		params[3] = &p3;

		thdParams[i] = params;
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

		

		void** retValue  = new void*[1];

		for(unsigned i = 0; i < imageNames.size();++i)
		{
			pthread_create(&imageThread[i], NULL, imageThreadFun, thdParams[i]);
		}
		
		for(unsigned i = 0; i < imageNames.size();++i)
		{
			pthread_join(imageThread[i], retValue);
			//systemPrint(INFO_SIMPLE,to_string(*((int*)*retValue)),THREAD_ID_IDENT_IMAGE);
			if (*((int*)*retValue)==1) {
				rename((*(((string**)(thdParams[i]))[2])).c_str(),imageNames[i].substr(33).c_str());
				imageNames.erase(imageNames.begin()+i);
				imageThread.erase(imageThread.begin() + i);
				thdParams.erase(thdParams.begin() + i);
				break;
				//rename("test.jpg","find.jpg");				
			}
			delete (int*)(*retValue);
		}

		delete retValue;

		this_thread::sleep_for(chrono::milliseconds(1000));
	}

	return NULL;
  }


 void* imageThreadFun(void* params){
 	string p0 = *((string*)(((void**)params)[0]));
 	string p1 = *((string*)(((void**)params)[1]));
 	string p2 = *((string*)(((void**)params)[2]));
 	string p3 = *((string*)(((void**)params)[3]));

 	int test = ident(p0,p1,p2,p3);

 	int* retValue = new int;
 	*retValue = test;

 	return (void*)retValue;
 }