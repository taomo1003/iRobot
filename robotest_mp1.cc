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

using namespace iRobot;
using namespace LibSerial;
using namespace std;

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
	
	srand(time(NULL));
	//send song
	Create::note_t note1(60,45);
	Create::note_t note2(Create::NO_NOTE,70);
	Create::song_t song = {note1, note2};
	robot.sendSongCommand(0,song);
	
	//LED flag
	bool LEDFlag = false;

	//led loop counter
	int LEDCounter = 0;
	
	//Back up flag
	bool BackUpFlag = false;
	
	//Back up counter
	int BackUpCounter = 0;
	
	//Rotate flag
	bool rotateFlag = false;
	
	//Rotate counter
	int rotateCounter = 0;
	
	//angle to rotate
	float angle = 0.0;
	
    // Let's stream some sensors.
    Create::sensorPackets_t sensors;
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    sensors.push_back (Create::SENSOR_BUTTONS);

    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;
    // Let's turn!
    int speed = 287;
    int ledColor = Create::LED_COLOR_GREEN;
    robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
    cout << "Sent Drive Command" << endl;

    short wallSignal = 0;
    while (!robot.playButton ())
    {
      if (robot.bumpLeft () || robot.bumpRight ()) {
        cout << "Bump !" << endl;
        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);		
		
	    if(!LEDFlag)
		  LEDCounter = 0;
		
		LEDFlag = true;

		BackUpFlag = true;
		BackUpCounter = 0;
      }
      short wallSignal = robot.wallSignal();
      if (wallSignal > 0) {
        cout << "Wall signal " << wallSignal << endl;
		//playsong
		int duration = 0;
		if(wallSignal>30)
			duration =5;
		else if(duration>20)
			duration = 10;
		else if(duration>10)
			duration = 15;
		else
			duration = 20;
		song[0].second = duration;
		song[1].second = duration;
		robot.sendSongCommand(0,song);
		robot.sendPlaySongCommand(0);
      }
	  if(LEDFlag)
	  {  
		  if(LEDCounter==0)
			robot.sendLedCommand(Create::LED_PLAY,Create::LED_COLOR_GREEN,Create::LED_INTENSITY_FULL);
		  
		  else if(LEDCounter==1)
		  {
			robot.sendLedCommand(Create::LED_ALL,Create::LED_COLOR_GREEN,Create::LED_INTENSITY_OFF);
		  }
		  
		  else if(LEDCounter==2)
			robot.sendLedCommand(Create::LED_ADVANCE,Create::LED_COLOR_RED,Create::LED_INTENSITY_FULL);
		  
		  else if(LEDCounter==3)
			robot.sendLedCommand(Create::LED_PLAY,Create::LED_COLOR_RED,Create::LED_INTENSITY_FULL);
		  
		  else if(LEDCounter==4)
			robot.sendLedCommand(Create::LED_ALL,Create::LED_COLOR_GREEN,Create::LED_INTENSITY_OFF);
		
		  else if(LEDCounter==5)
			robot.sendLedCommand(Create::LED_ADVANCE,Create::LED_COLOR_GREEN,Create::LED_INTENSITY_FULL);
		
		  if(LEDCounter >= 10)
		  {
			LEDCounter = (LEDCounter-9)%6;
		  }
		  else
		  {
		  	LEDCounter += 10;
		  }		
	  }
	  if(BackUpFlag)
	  {
		  if(BackUpCounter == 0)
			robot.sendDriveCommand(-165, Create::DRIVE_STRAIGHT);
		
		  else if(BackUpCounter == 22)
		  {
			LEDFlag = false;
			BackUpFlag = false;
			rotateFlag = true;
			rotateCounter = 0;
			
			angle = (rand()%121+120)/4.90452;
			
			robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);	
			
            Camera.grab();
            Camera.retrieve (bgr_image);
            cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
            cv::imwrite("irobot_image.jpg", rgb_image);
            cout << "Taking photo" << endl;
		  }	
		  
		  BackUpCounter++;
	  }
	  
	  if(rotateFlag)
	  {
		if(rotateCounter==0)
           robot.sendDriveCommand (107, Create::DRIVE_INPLACE_CLOCKWISE);
	    else if(angle < 1)
		{
		  this_thread::sleep_for(chrono::milliseconds(static_cast<int>(angle*100)));
		  robot.sendDriveCommand (0, Create::DRIVE_INPLACE_CLOCKWISE);
		  robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
		  rotateFlag = false;
		}
		
		rotateCounter++;
		angle--;
 
	  }

      // You can add more commands here.
      this_thread::sleep_for(chrono::milliseconds(100));
    }
    
    cout << "Play button pressed, stopping Robot" << endl;
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
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
}
