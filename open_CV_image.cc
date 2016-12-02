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
	bool whileLoop = stop_running_thread(THREAD_ID_IDENT_IMAGE);
	raspicam::RaspiCam_Cv* Camera = ((raspicam::RaspiCam_Cv*) ((void**)params)[0]);
	Create* robot = (Create*)((void**)params)[1];
	cv::Mat rgb_image, bgr_image;

	vector<string> imageName;
	vector<int> count_vec;

	imageName.push_back("magic-lamp");
	//if changing this low to high resolution change rename function below.
	imageName[0] =(string)"./object-pictures/low-resolution/" + imageName[0] + (string)"-600.jpg";




	int count = 0;


	while(!whileLoop)
	{	
		auto start_time = std::chrono::system_clock::now();
		auto deadline = start_time + std::chrono::milliseconds(6000); 

		lockMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);
		robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		this_thread::sleep_for(chrono::milliseconds(500));

		Camera->grab();
		Camera->retrieve (bgr_image);
		cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		cv::imwrite("pictures/irobot_image.jpg", rgb_image);
		systemPrint(INFO_NONE, "Taking photo", THREAD_ID_IDENT_IMAGE);

		
		unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);
		
		int result = 0;
		
		if (imageName.size() != 0)
		{
			result = ident(imageName,"pictures/irobot_image.jpg");
		}
		
		rename(((string)"pictures/irobot_image.jpg").c_str() , ("pictures/"+to_string(count)+(string)".jpg").c_str());

		count_vec.push_back(count);
		
		count++;

		if (result == 1) {
			lockMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);
		    robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			// this_thread::sleep_for(chrono::milliseconds(500));

			// Camera->grab();
			// Camera->retrieve (bgr_image);
			// cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
			// cv::imwrite("pictures/irobot_image.jpg", rgb_image);
			// systemPrint(INFO_NONE, "Taking photo", THREAD_ID_IDENT_IMAGE);

			// result = ident(imageName,"pictures/irobot_image.jpg");
			
			if (result == 1)
			{
				robot->sendLedCommand(Create::LED_NONE,Create::LED_COLOR_RED,Create::LED_INTENSITY_FULL);
				this_thread::sleep_for(chrono::milliseconds(2000));
				robot->sendLedCommand(Create::LED_NONE,Create::LED_COLOR_RED,Create::LED_INTENSITY_OFF);
			}
			else
			{
				endMission(THREAD_ID_IDENT_IMAGE, robot);
			}
			
			unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_IDENT_IMAGE);
		}
		this_thread::sleep_until(deadline);
		whileLoop = stop_running_thread(THREAD_ID_IDENT_IMAGE);
	}
	
	short batchSize = 6;
	for (int i = 0; i < count; i += batchSize)
	{
		thread_manager gTheThreadManager;
		
		for (int j = 0; j < batchSize; ++j)
		{
			if (i+j < count)
				gTheThreadManager.create_new_thread(indent_post, count, (void*)(&count_vec[i+j]), 70,false);
		}
		
		gTheThreadManager.join_all_threads(false);
	}

	return nullptr;
}

void* indent_post(void* params){
	vector<string> imageNames;

	int count = *((int*)params);

	imageNames.push_back("ancient-lamp");
	imageNames.push_back("audio-cassette");
	imageNames.push_back("mammoth");
	imageNames.push_back("mayan-calendar");
	imageNames.push_back("mjolnir-hammer");
	imageNames.push_back("one-ring");	
	imageNames.push_back("pueblo-pot");
	imageNames.push_back("roman-glass");
	imageNames.push_back("willow-plate");

	for(unsigned i = 0; i < imageNames.size(); ++i)
	{
	//if changing this low to high resolution change rename function below.
		imageNames[i] =(string)"./object-pictures/low-resolution/" + imageNames[i] + (string)"-600.jpg";
	}

	systemPrint(INFO_NONE, "PostProcessing Image"+to_string(count) , THREAD_ID_IDENT_IMAGE);
	ident(imageNames,("pictures/"+to_string(count)+(string)".jpg").c_str());
}

