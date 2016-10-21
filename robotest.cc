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

using namespace iRobot;
using namespace LibSerial;
using namespace std;

enum THREAD_ID:int
{
	THREAD_ID_NAV,
	N_THREADS
};

const int WALL_SENSOR_WALL = 200;
const int ACCEPTED_OFF_MAX = 5;

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
	// 0-31 priorities
	//SetThreadPriority(threads[THREAD_ID_NAV],31);
	
	for(int i=0; i<N_THREADS;i++)
	{
		threads[i].join();
	}
	
	robot.sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
	
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

const int ROTATE_SPEED = 200;

int find_max_wall_signal(Create* robot)
{
	short wallSignal;
	short maxWallSignal = -1;
	short prevWallSignal = 0;
	bool inc = false;
	

	  cout << "Spinning to Parallel" << endl;
	  
	  robot->sendDriveCommand (ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  for (int i = 0; i < 400; ++i)
	  {
		this_thread::sleep_for(chrono::milliseconds(10));
		prevWallSignal = wallSignal;
		wallSignal = robot->wallSignal();
		cout << wallSignal << endl;
		if (wallSignal - prevWallSignal >= 4)
		{
			
			inc = true;
		}
		if (inc && wallSignal < prevWallSignal && wallSignal<40)
		{
			maxWallSignal = prevWallSignal;
			cout << "Found local maximum: " << maxWallSignal << endl;
			break;
		}
	  }
	  robot->sendDriveCommand (0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  cout << "Done." << endl;
	  
	  return maxWallSignal;
}
/*
void spin_to_max(Create* robot, short& maxWallSignal)
{
	cout << "Spinning to be parallel." << endl;
	robot->sendDriveCommand (ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	int counter = 0;
	while(robot->wallSignal() < maxWallSignal - ACCEPTED_OFF_MAX)
	  {
		this_thread::sleep_for(chrono::milliseconds(100));
		if (counter >= 40)
		{
			cout << "Recalibrating" << endl;
			maxWallSignal = find_max_wall_signal(robot);
			counter = 0;
		}
		counter++;
	  }
	  this_thread::sleep_for(chrono::milliseconds(100));
	  robot->sendDriveCommand (0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
	  
	  cout << "iRobot is now parallel" << endl;
}*/

void nav_test(Create* robot)
{	
	short wallSignal = 0;
	short prevWallSignal =0;
	short maxWallSignal = find_max_wall_signal(robot);
	short wallSignalFail = 0;
	  
	//spin_to_max(robot, maxWallSignal);
	

	
	robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
	
	while (!robot->playButton ())
    {
      wallSignal = robot->wallSignal();
	  
	  if(robot->bumpLeft () || robot->bumpRight ())
	  {
		  cout << "Bump." << endl;
		  robot->sendDriveCommand(-207, Create::DRIVE_STRAIGHT);
		  this_thread::sleep_for(chrono::milliseconds(150));
		  robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
	  
		  maxWallSignal = find_max_wall_signal(robot);
		  /*
		  cout << "Spinning to be parallel." << endl;
		  while(robot->wallSignal() < maxWallSignal - ACCEPTED_OFF_MAX)
		  {
			  robot->sendDriveCommand (107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
			this_thread::sleep_for(chrono::milliseconds(10));
		  }*/
		  robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
	  }
	  
	  if(wallSignal<4 && maxWallSignal>20 && prevWallSignal<4)
	  {
		cout<<"turning corner"<<endl;
		this_thread::sleep_for(chrono::milliseconds(1000));
		robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		robot->sendDriveCommand (ROTATE_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
		this_thread::sleep_for(chrono::milliseconds(700));
		robot->sendDriveCommand (0, Create::DRIVE_INPLACE_CLOCKWISE);
		robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
		this_thread::sleep_for(chrono::milliseconds(1000));
		maxWallSignal = find_max_wall_signal(robot);
		robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
	  }

	  if(wallSignal <= maxWallSignal - ACCEPTED_OFF_MAX)
	  {
		  //++wallSignalFail;
		robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		robot->sendDriveCommand (ROTATE_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
		this_thread::sleep_for(chrono::milliseconds(100));
		robot->sendDriveCommand (0, Create::DRIVE_INPLACE_CLOCKWISE);
		robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);

	  }
	  else if(wallSignal >= maxWallSignal + ACCEPTED_OFF_MAX)
	  {
		  //--wallSignalFail;
		robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		robot->sendDriveCommand (ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		this_thread::sleep_for(chrono::milliseconds(100));
		robot->sendDriveCommand (0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);	
		robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);		
	  }
	  /*else
	  {
		  if (wallSignalFail < 0)
		  {
			  ++wallSignalFail;
		  }
		  else if (wallSignalFail > 0)
		  {
			  --wallSignalFail;
		  }
	  }
	  if (wallSignalFail <= -5 || wallSignalFail >= 5)
	  {
		  cout << "Recalibrating" << endl;
	  	maxWallSignal = find_max_wall_signal(robot);
		robot->sendDriveCommand(207, Create::DRIVE_STRAIGHT);
		  wallSignalFail = 0;
	  }*/
	  prevWallSignal = wallSignal;
	  this_thread::sleep_for(chrono::milliseconds(100));
	}
	return;
  }