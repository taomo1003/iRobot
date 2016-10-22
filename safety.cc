#include "safety.hh"

void* safety(void* parms)
{	
	Create* robot = (Create*)parms;
	if(robot==nullptr)
	{
		cerr<<"robot not defined"<<endl;
		return nullptr;
	}
	
	bool mtxLocked = false;
	bool overcurrnt = false;
	
	Create::note_t note1(60,45);
	Create::note_t note2(Create::NO_NOTE,70);
	Create::song_t song = {note1, note2};
	robot->sendSongCommand(0,song);
				
	while(true)
	{
		//if a safety sencer is fired stop the robot.
		if(robot->wheeldropLeft()         || robot->wheeldropRight()        || robot->wheeldropCaster() ||
		   robot->rightWheelOvercurrent() || robot->leftWheelOvercurrent()  ||
		   robot->cliffLeftSignal()		  || robot->cliffFrontLeftSignal()	||
		   robot->cliffRightSignal()	  || robot->cliffFrontRightSignal()									)
		{
			if(robot->rightWheelOvercurrent() || robot->leftWheelOvercurrent())
			{
				overcurrnt=true;
			}
			if (!mtxLocked)
			{
				cout << "Safety Locked." << endl;
				lockMtx(MUTEX_ID_SAFETY);
				mtxLocked = true;
				robot->sendDriveCommand(0.0, Create::DRIVE_STRAIGHT);
			}
			robot->sendPlaySongCommand(0);
		}
		
		if(robot->playButton())
		{
			overcurrnt=false;
		}
		
		//if no saftey sencers are fired start the robot
		if(!robot->wheeldropLeft()         && !robot->wheeldropRight()        && !robot->wheeldropCaster() &&
		   !overcurrnt					   &&
		   !robot->cliffLeftSignal()	   && !robot->cliffFrontLeftSignal()  &&
		   !robot->cliffRightSignal()	   && !robot->cliffFrontRightSignal()									)
		{
			if (mtxLocked)
			{
				cout << "Safety Unlocked." << endl;
				unlkMtx(MUTEX_ID_SAFETY);
				mtxLocked = false;
			}
		}
		
		this_thread::sleep_for(chrono::milliseconds(100));
	}
}