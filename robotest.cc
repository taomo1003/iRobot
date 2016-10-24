#include "robotest.hh"
#include "open_CV_image.hh"
#include "robotest.hh"
#include "nav.hh"
#include "safety.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t cameraMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t safetyMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t printMutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t serialMutex  = PTHREAD_MUTEX_INITIALIZER;



	thread_manager gTheThreadManager;

int main ()
{
	char serial_loc[] = "/dev/ttyUSB0";
	
	systemPrint(INFO_NONE, "Main", THREAD_ID_MAIN);
	systemPrint(INFO_NONE, "Nav", THREAD_ID_NAV);
	systemPrint(INFO_NONE, "Ident Image", THREAD_ID_IDENT_IMAGE);
	systemPrint(INFO_NONE, "Ident Contour", THREAD_ID_IDENT_CONTOUR);
	systemPrint(INFO_NONE, "Safety", THREAD_ID_SAFETY);
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
    sensors.push_back (Create::SENSOR_BUTTONS);
	sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    
    robot.sendStreamCommand (sensors);
	
	systemPrint(INFO_NONE, "Sent Stream Command", THREAD_ID_MAIN);

	systemPrint(INFO_NONE, "", THREAD_ID_MAIN);
	systemPrint(INFO_NONE, "Charge: " + to_string(robot.batteryCharge()), THREAD_ID_MAIN);
	systemPrint(INFO_NONE, "", THREAD_ID_MAIN);
	
	this_thread::sleep_for(chrono::milliseconds(3000));
	
    void** paramsForOpenCVImage = new void*[2];

    paramsForOpenCVImage[0] = &Camera;
    paramsForOpenCVImage[1] = &robot;
	
	gTheThreadManager.create_new_thread(nav_test, THREAD_ID_NAV, (void*)&robot, 25);
	
	gTheThreadManager.create_new_thread(open_CV_image, THREAD_ID_IDENT_IMAGE, (void*)paramsForOpenCVImage, 24);

	//gTheThreadManager.create_new_thread(safety, THREAD_ID_SAFETY, (void*)&robot, 31);

	gTheThreadManager.join_all_threads();
	
	pthread_mutex_destroy ( &safetyMutex );
	pthread_mutex_destroy ( &cameraMutex );
	pthread_mutex_destroy ( &printMutex  );
	pthread_mutex_destroy ( &serialMutex );


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