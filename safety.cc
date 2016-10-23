#include "safety.hh"

void* safety(void* parms)
{	
	Create* robot = (Create*)parms;
	if(robot==nullptr)
	{
		systemPrint(INFO_NONE, "robot not defined", THREAD_ID_SAFETY);
		return nullptr;
	}
	
	bool mtxLocked = false;
	bool overcurrent = false;
	
	Create::note_t note1(60,45);
	Create::note_t note2(Create::NO_NOTE,70);
	Create::song_t song = {note1, note2};
	
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
	robot->sendSongCommand(0,song);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
				
	while(true)
	{
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
		bool SafteySensorFired;
		SafteySensorFired = robot->wheeldropLeft();
		SafteySensorFired |=robot->wheeldropRight();
		SafteySensorFired |=robot->wheeldropCaster();//might not need this one

		bool overcurrentsensor = false;
		//bool overcurrentsensor = robot->rightWheelOvercurrent();
		//overcurrentsensor |=robot->leftWheelOvercurrent();
		
		//SafteySensorFired |=robot->cliffLeftSignal();
		//SafteySensorFired |=robot->cliffFrontLeftSignal();
		//SafteySensorFired |=robot->cliffRightSignal();
		//SafteySensorFired |=robot->cliffFrontRightSignal();
		
		bool robotButton = robot->playButton();
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);

		//if a safety sencer is fired stop the robot.
		if(SafteySensorFired || overcurrentsensor)
		{
			if(overcurrentsensor)
			{
				overcurrent=true;
			}
			if (!mtxLocked)
			{
				lockMtx(MUTEX_ID_SAFETY, THREAD_ID_SAFETY);
				mtxLocked = true;
				
				lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
				robot->sendDriveCommand(0.0, Create::DRIVE_STRAIGHT);
				unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
			}
			lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
			robot->sendPlaySongCommand(0);
			unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
		}
		
		if(robotButton)
		{
			overcurrent=false;
		}
		
		//if no saftey sencers are fired start the robot
		if(!SafteySensorFired && !overcurrent)
		{
			if (mtxLocked)
			{
				unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_SAFETY);
				mtxLocked = false;
			}
		}
		
		this_thread::sleep_for(chrono::milliseconds(100));
	}
}