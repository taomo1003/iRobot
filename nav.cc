#include "nav.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

void* nav_test(void* parms)
{	
	Create* robot = (Create*)parms;
	if(robot==nullptr)
	{
		cerr<<"robot not defined"<<endl;
		return nullptr;
	}
	short wallSignal = 0;
	short prevWallSignal =0;
	short maxWallSignal = find_max_wall_signal(robot);
	short wallSignalFail = 0;
	  
	//spin_to_max(robot, maxWallSignal);
	

	
	sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
	
	while (!robot->playButton ())
    {
      wallSignal = robot->wallSignal();
	  
	  if(robot->bumpLeft () || robot->bumpRight ())
	  {
		  cout << "Bump." << endl;
		  sendDriveCommand(robot,-207, Create::DRIVE_STRAIGHT);
		  this_thread::sleep_for(chrono::milliseconds(150));
		  sendDriveCommand(robot,0, Create::DRIVE_STRAIGHT);
	  
		  maxWallSignal = find_max_wall_signal(robot);
		  /*
		  cout << "Spinning to be parallel." << endl;
		  while(robot->wallSignal() < maxWallSignal - ACCEPTED_OFF_MAX)
		  {
			  sendDriveCommand(robot,107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
			this_thread::sleep_for(chrono::milliseconds(10));
		  }*/
		  sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
	  }
	  /*
	  if(wallSignal<4 && maxWallSignal>20 && prevWallSignal<4)
	  {
		cout<<"turning corner"<<endl;
		this_thread::sleep_for(chrono::milliseconds(1000));
		sendDriveCommand(robot,0, Create::DRIVE_STRAIGHT);
		sendDriveCommand(robot,ROTATE_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
		this_thread::sleep_for(chrono::milliseconds(700));
		sendDriveCommand(robot,0, Create::DRIVE_INPLACE_CLOCKWISE);
		sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
		this_thread::sleep_for(chrono::milliseconds(1000));
		maxWallSignal = find_max_wall_signal(robot);
		sendDriveCommand(robot, 207, Create::DRIVE_STRAIGHT);
	  }
		*/
	  if(wallSignal <= maxWallSignal - ACCEPTED_OFF_MAX)
	  {
		  //++wallSignalFail;
		sendDriveCommand(robot,ROTATE_SPEED, -ROTATE_RADIUS);

	  }
	  else if(wallSignal >= maxWallSignal + ACCEPTED_OFF_MAX)
	  {
		  //--wallSignalFail;
		//sendDriveCommand(robot,ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		sendDriveCommand(robot,ROTATE_SPEED, ROTATE_RADIUS);
	  }
	  else
	  {
		 sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);	
	  }
	  this_thread::sleep_for(chrono::milliseconds(100));
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
		sendDriveCommand(robot,207, Create::DRIVE_STRAIGHT);
		  wallSignalFail = 0;
	  }*/
	  prevWallSignal = wallSignal;
	  this_thread::sleep_for(chrono::milliseconds(100));
	}
	return nullptr;
  }

void sendDriveCommand(Create* robot, const int speed, Create::DriveCommand direction)
{
	sem_wait(movement);
	robot->sendDriveCommand(speed, direction);
	sem_post(movement);
}

void sendDriveCommand(Create* robot, const int speed, short radius)
{
	cout << "drive function" << endl;
	sem_wait(movement);
	cout << "   sem lock" << endl;
	robot->sendDriveCommand(speed, radius);
	sem_post(movement);
	cout << "   sem unlock" << endl;
}

int find_max_wall_signal(Create* robot)
{
	short wallSignal;
	short maxWallSignal = -1;
	short prevWallSignal = 0;
	bool inc = false;
	

	  cout << "Spinning to Parallel" << endl;
	  
	  sendDriveCommand(robot,ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
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
	  sendDriveCommand(robot,0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  cout << "Done." << endl;
	  
	  return maxWallSignal;
}