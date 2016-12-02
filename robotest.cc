#include "robotest.hh"
#include "open_CV_image.hh"
#include "robotest.hh"
#include "nav.hh"
#include "safety.hh"
#include "external.hh"


using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t cameraMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t safetyMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t printMutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t serialMutex  = PTHREAD_MUTEX_INITIALIZER;
thread_manager gTheThreadManager;
sem_t kill_all_threads;

int main ()
{
	char serial_loc[] = "/dev/ttyUSB0";
	
	sem_init(&kill_all_threads, 0, 0);
	
	for (int i = 0; i < N_THREADS; ++i)
	{
		systemPrint(INFO_NONE, threadIdToString((THREAD_ID)i), (THREAD_ID)i);
	}
	
	systemPrint(INFO_NONE, "", THREAD_ID_MAIN);
	
	this_thread::sleep_for(chrono::milliseconds(1000));
	
    try
    {
		raspicam::RaspiCam_Cv Camera;
		cv::Mat rgb_image, bgr_image;
		if (!Camera.open()) {
			systemPrint(INFO_NONE, "Error opening the camera", THREAD_ID_MAIN);
		  return -1;
    }
    systemPrint(INFO_NONE, "Opened Camera", THREAD_ID_MAIN);
    SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
	systemPrint(INFO_NONE, "Opened Serial Stream to /dev/ttyUSB0", THREAD_ID_MAIN);
    this_thread::sleep_for(chrono::milliseconds(1000));
    Create robot(stream);
	systemPrint(INFO_NONE, "Created iRobot Object", THREAD_ID_MAIN);
    robot.sendFullCommand();
	systemPrint(INFO_NONE, "Setting iRobot to Full Mode", THREAD_ID_MAIN);
    this_thread::sleep_for(chrono::milliseconds(1000));
	systemPrint(INFO_NONE, "Robot is ready", THREAD_ID_MAIN);
	
	// Let's stream some sensors.
    Create::sensorPackets_t sensors;
    //sensors.push_back(Create::SENSOR_GROUP_1);
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_LEFT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_RIGHT_SIGNAL);
    sensors.push_back(Create::SENSOR_OVERCURRENTS);
    sensors.push_back (Create::SENSOR_BUTTONS);


	//sensors.push_back(Create::SENSOR_DISTANCE);
	//sensors.push_back(Create::SENSOR_ANGLE);
	
    
    robot.sendStreamCommand (sensors);

	systemPrint(INFO_NONE, "Sent Stream Command", THREAD_ID_MAIN);
	
	robot.sendSensorsCommand(Create::SENSOR_BATTERY_CHARGE);
	robot.updateSensors();

	systemPrint(INFO_NONE, "", THREAD_ID_MAIN);
	systemPrint(INFO_NONE, "Charge: " + to_string(((unsigned short)robot.batteryCharge())/65535.0F), THREAD_ID_MAIN);
	systemPrint(INFO_NONE, "", THREAD_ID_MAIN);
	
	this_thread::sleep_for(chrono::milliseconds(3000));
	
    void** paramsForOpenCVImage = new void*[2];

    paramsForOpenCVImage[0] = &Camera;
    paramsForOpenCVImage[1] = &robot;
		
		
	gTheThreadManager.create_new_thread(safety, THREAD_ID_SAFETY, (void*)&robot, 95);
	
	gTheThreadManager.create_new_thread(nav_test, THREAD_ID_NAV, (void*)&robot, 94);
	
	gTheThreadManager.create_new_thread(open_CV_image, THREAD_ID_IDENT_IMAGE, (void*)paramsForOpenCVImage, 93);
	
	gTheThreadManager.create_new_thread(external, THREAD_ID_EXTERNAL, (void*)&robot, 60);
	
	sched_param priority;
	priority.sched_priority = 95;
 	pthread_setschedparam(pthread_self(), SCHED_FIFO,&priority);

 	auto two_min_count = chrono::system_clock::now();

	bool run = true;
	while (run)
	{
		auto new_time_2_min = chrono::system_clock::now();
		auto start_time = std::chrono::system_clock::now();
		auto deadline = start_time + std::chrono::milliseconds(100);
		
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_MAIN);


		run = (chrono::milliseconds(115000) > (chrono::duration_cast<chrono::milliseconds>(new_time_2_min - two_min_count)) &&
			   (!robot.playButton()) &&
			   (!stop_running_thread(THREAD_ID_MAIN)));
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_MAIN);
		
		this_thread::sleep_until(deadline);
	}
	 auto stopTime = chrono::system_clock::now();
	systemPrint(INFO_NONE, "Mission Time:" +to_string((float)chrono::duration_cast<chrono::milliseconds>(stopTime - two_min_count).count() / 1000.0f), THREAD_ID_MAIN);
	endMission(THREAD_ID_MAIN,&robot);
	gTheThreadManager.join_all_threads();
	systemPrint(INFO_NONE, "all threads joined", THREAD_ID_MAIN);

	pthread_mutex_destroy ( &safetyMutex );
	pthread_mutex_destroy ( &cameraMutex );
	pthread_mutex_destroy ( &printMutex  );
	pthread_mutex_destroy ( &serialMutex );
	sem_destroy(&kill_all_threads);

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

bool stop_running_thread(const THREAD_ID Id)
{
	int value;

	sem_getvalue(&kill_all_threads, &value);

	systemPrint(INFO_ALL, "sem value:" + to_string(value), Id);

	if(value==0)
		return false;
	else
		return true;
}

void lockMtx(const MUTEX_ID MtxID, const THREAD_ID Id)
{
	if (MtxID == MUTEX_ID_SAFETY)
	{
		pthread_mutex_lock(&safetyMutex);
		systemPrint(INFO_ALL, "Locked Safety Mutex", Id);
	}
	else if (MtxID == MUTEX_ID_CAMERA)
	{
		pthread_mutex_lock(&cameraMutex);
		systemPrint(INFO_ALL, "Locked Camera Mutex", Id);
	}
	else if (MtxID == MUTEX_ID_PRINT)
	{
		pthread_mutex_lock(&printMutex);
	}
	else if (MtxID == MUTEX_ID_SERIAL)
	{
		pthread_mutex_lock(&serialMutex);
		systemPrint(INFO_ALL, "Locked Serial Mutex", Id);
	}
	else
	{
		systemPrint(INFO_NONE, "void unlkMtx(const MUTEX_ID MtxID) :::: Undefined MtxID", Id);
	}
}

void unlkMtx(const MUTEX_ID MtxID, const THREAD_ID Id)
{
	if (MtxID == MUTEX_ID_SAFETY)
	{
		pthread_mutex_unlock(&safetyMutex);
		systemPrint(INFO_ALL, "Unlocked Safety Mutex", Id);
	}
	else if (MtxID == MUTEX_ID_CAMERA)
	{
		pthread_mutex_unlock(&cameraMutex);
		systemPrint(INFO_ALL, "Unlocked Camera Mutex", Id);
	}
	else if (MtxID == MUTEX_ID_PRINT)
	{
		pthread_mutex_unlock(&printMutex);
	}
	else if (MtxID == MUTEX_ID_SERIAL)
	{
		pthread_mutex_unlock(&serialMutex);
		systemPrint(INFO_ALL, "Unlocked Serial Mutex", Id);
	}
	else
	{
		systemPrint(INFO_NONE, "void unlkMtx(const MUTEX_ID MtxID) :::: Undefined MtxID", Id);
	}
}
void endMission(const THREAD_ID id, Create* r)
{
	sem_post(&kill_all_threads);
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_MAIN); //here we are
	r->sendLedCommand(Create::LED_NONE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_MAIN);
	systemPrint(INFO_NONE,"Mission Ended By " + threadIdToString(id), id);
}
void systemPrint(const INFO_LEVEL lvl,  const string s, const THREAD_ID id)
{
	if (id != N_THREADS)
	{
		if (lvl <= gPrintDebug)
		{
			const string colors[] =
			{
				"\x1b[31m", // ANSI_COLOR_RED
				"\x1b[32m",//ANSI_COLOR_GREEN
				"\x1b[33m",//ANSI_COLOR_YELLOW
				"\x1b[34m",//ANSI_COLOR_BLUE
				"\x1b[35m",//ANSI_COLOR_MAGENTA
				"\x1b[36m",//ANSI_COLOR_CYAN
				"\x1b[0m"  //ANSI_COLOR_RESET
			};
			
			try{
			lockMtx(MUTEX_ID_PRINT, N_THREADS);
			printf((colors[id]+s+colors[id]+"\n").c_str());
			unlkMtx(MUTEX_ID_PRINT, N_THREADS);
			}catch(ios_base::failure e)
			{
				printf((colors[id]+e.what()+colors[id]+"\n").c_str());
			}
		}
	}
}

string threadIdToString(const THREAD_ID Id)
{
	switch(Id)
	{
	case THREAD_ID_MAIN:
		return "Main";
	case THREAD_ID_NAV:
		return "Nav";
	case THREAD_ID_SAFETY:
		return "Safety";
	case THREAD_ID_IDENT_IMAGE:
		return "Ident Image";
	case THREAD_ID_EXTERNAL:
		return "External";
	default:
		return "";
	}
}