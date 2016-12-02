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
	bool ocL = false;
	bool ocR = false;
	short overcurrentsensor = 0;
	
	Create::note_t note1(60,45);
	Create::note_t note2(Create::NO_NOTE,70);
	Create::song_t song = {note1, note2};
	
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
	robot->sendSongCommand(0,song);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
			
	bool loop = stop_running_thread(THREAD_ID_SAFETY);
	while(!loop)
	{
		auto start_time = std::chrono::system_clock::now();
		auto deadline = start_time + std::chrono::milliseconds(16); // SMALLER

		loop = stop_running_thread(THREAD_ID_SAFETY);
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
		bool SafteySensorFired;
		SafteySensorFired = robot->wheeldropLeft();
		SafteySensorFired |=robot->wheeldropRight();
		SafteySensorFired |=robot->wheeldropCaster();//might not need this one

		ocR = robot->rightWheelOvercurrent();
		ocL = robot->leftWheelOvercurrent();
		
		unsigned short cliffL = robot->cliffLeftSignal();
		unsigned short cliffR = robot->cliffFrontLeftSignal();
		unsigned short cliffLF = robot->cliffRightSignal();
		unsigned short cliffRF = robot->cliffFrontRightSignal();
		
		bool robotButton = robot->advanceButton();
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
		if(ocR)
			overcurrentsensor++;
		else if(overcurrentsensor >0)
			overcurrentsensor--;
		
		if(ocL)
			overcurrentsensor++;
		else if(overcurrentsensor >0)
			overcurrentsensor--;
		
		if(cliffL<5 || cliffR<5 || cliffLF<5|| cliffRF<5 || overcurrentsensor>5)
		{
			SafteySensorFired = true;
		}
		//if a safety sencer is fired stop the robot.
		if(SafteySensorFired ||overcurrent)
		{
			if(overcurrentsensor > 5)
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
				systemPrint(INFO_SIMPLE, "Stopped Robot", THREAD_ID_SAFETY);
			}
			lockMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
			robot->sendPlaySongCommand(0);
			unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_SAFETY);
		}
		
		if(robotButton)
		{
			overcurrent=false;
			overcurrentsensor=0;
		}
		
		//if no saftey sencers are fired start the robot
		if(!SafteySensorFired && !overcurrent)
		{
			if (mtxLocked)
			{
				unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_SAFETY);
				mtxLocked = false;
				systemPrint(INFO_SIMPLE, "Freed Robot", THREAD_ID_SAFETY);
			}
		}
		
		this_thread::sleep_until(deadline);
	}
	
	if (mtxLocked)
	{
		unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_SAFETY);
		mtxLocked = false;
		systemPrint(INFO_SIMPLE, "Freed Robot", THREAD_ID_SAFETY);
	}
}