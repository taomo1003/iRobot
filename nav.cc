#include "nav.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

void* nav_test(void* parms)
{	
	Create* robot = (Create*)parms;
	if(robot==nullptr)
	{
		systemPrint(INFO_NONE, "Robot Not Defined", THREAD_ID_NAV);
		
		return nullptr;
	}
	short wallSignal = 0;
	short prevWallSignal =0;
	short maxWallSignal = 0;
	short wallSignalFail = 0;	
	
	bool loop = false;
	while(!loop)
	{
		sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
		loop = robot->bumpLeft()|robot->bumpLeft();
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	}
	loop = true;
	while (loop)
    {
	  lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
      wallSignal = robot->wallSignal();
	  bool bBumpLeft = robot->bumpLeft();
	  bool bBumpRight = robot->bumpRight();
	  loop = !robot->playButton ();
	  unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	  
	  systemPrint(INFO_ALL, to_string(wallSignal), THREAD_ID_NAV);
	  
	  
	  if( bBumpLeft || bBumpRight )
	  {
		  systemPrint(INFO_SIMPLE, "Bump", THREAD_ID_NAV);
		  sendDriveCommand(robot,-207, Create::DRIVE_STRAIGHT);
		  this_thread::sleep_for(chrono::milliseconds(150));
		  sendDriveCommand(robot,0, Create::DRIVE_STRAIGHT);
	  
		  maxWallSignal = find_max_wall_signal(robot);

		  sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
	  }
	  if(wallSignal <= maxWallSignal - ACCEPTED_OFF_MAX)
	  {
		sendDriveCommand(robot,ROTATE_SPEED, -ROTATE_RADIUS);
	  }
	  else if(wallSignal >= maxWallSignal + ACCEPTED_OFF_MAX)
	  {
		sendDriveCommand(robot,ROTATE_SPEED, ROTATE_RADIUS);
	  }
	  else
	  {
		 sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);	
	  }
	  this_thread::sleep_for(chrono::milliseconds(100));
	  prevWallSignal = wallSignal;
	}
	return nullptr;
  }

void sendDriveCommand(Create* robot, const int speed, Create::DriveCommand direction)
{
	lockMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	robot->sendDriveCommand(speed, direction);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
}

void sendDriveCommand(Create* robot, const int speed, short radius)
{	
	lockMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	robot->sendDriveCommand(speed, radius);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
}

int find_max_wall_signal(Create* robot)
{
	short wallSignal;
	short maxWallSignal = -1;
	short prevWallSignal = 0;
	bool inc = false;
	
	systemPrint(INFO_SIMPLE, "Spinning to Parallel", THREAD_ID_NAV);
	  
	  for (int i = 0; i < 400; ++i)
	  {
	    sendDriveCommand(robot,ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		this_thread::sleep_for(chrono::milliseconds(10));
		prevWallSignal = wallSignal;
		
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
		wallSignal = robot->wallSignal();
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);

		
		//cout << wallSignal << endl;
		if (wallSignal - prevWallSignal >= 4)
		{
			inc = true;
		}
		if (inc && wallSignal < prevWallSignal && wallSignal<40)
		{
			maxWallSignal = prevWallSignal;
			systemPrint(INFO_SIMPLE, "Found local maximum " + maxWallSignal, THREAD_ID_NAV);
			break;
		}
	  }
	  systemPrint(INFO_SIMPLE, "Done", THREAD_ID_NAV);
	  sendDriveCommand(robot,0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  
	  return maxWallSignal;
}