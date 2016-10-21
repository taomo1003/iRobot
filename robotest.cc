#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <time.h>
#include <thread>
#include "robovision.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

enum THREAD_ID:int
{
	THREAD_ID_NAV,
	IDENTIFATION_IMAGE,
	N_THREADS
};

void nav_test(Create* d);

int main ()
{
	char serial_loc[] = "/dev/ttyUSB0";
	
    try
    {
		raspicam::RaspiCam_Cv Camera;
		cv::Mat rgb_image, bgr_image;
		if (!Camera.open()) {
		  cerr << "Error opening the camera" << endl;
		  return -1;
    }
    cout << "Opened Camera" << endl;
    SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
    cout << "Opened Serial Stream to" << serial_loc << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    Create robot(stream);
    cout << "Created iRobot Object" << endl;
    robot.sendFullCommand();
    cout << "Setting iRobot to Full Mode" << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    cout << "Robot is ready" << endl;
	
	// Let's stream some sensors.
    Create::sensorPackets_t sensors;
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    sensors.push_back (Create::SENSOR_BUTTONS);

    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;
	
	thread threads[N_THREADS];
	
	threads[THREAD_ID_NAV] = thread(nav_test, &robot);

	// threads[IDENTIFATION_IMAGE] = thread(robovision::ident, 
	// 	"./object-pictures/ancient-lamp.jpg", 
	// 	"./object-pictures/low-resolution/ancient-lamp-600.jpg",
	// 	"./object-pictures/test.jpg",
	// 	1);

	// 0-31 priorities
	//SetThreadPriority(threads[THREAD_ID_NAV],31);
	
	for(int i=0; i<N_THREADS;i++)
	{
		threads[i].join();
	}
	
	}
	catch (InvalidArgument& e)
  {
    cerr << e.what () << endl;
    return 3;
  }
  catch (CommandNotAvailable& e)
  {
    cerr << e.what () << endl;
    return 4;
  }
	
	return 0;
}


void nav_test(Create* robot)
{	
	short wallSignal = 0;
	string p1 = "./object-pictures/ancient-lamp.jpg";
	string p2 = "./object-pictures/low-resolution/ancient-lamp-600.jpg";
	string p3 = "./object-pictures/test.jpg";
	string p4 = "1.0";
	int test = ident(p1,p2,p3,p4);
	
	while (!robot->playButton ())
    {
      //Todo do something here

	}
	return;
  }