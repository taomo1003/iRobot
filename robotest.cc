#include "robotest.hh"
//#include "nav.hh"
#include "open_CV_image.hh"
//#include "open_CV_contour.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t movement = PTHREAD_MUTEX_INITIALIZER;

	thread_manager gTheThreadManager;

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
	
	//gTheThreadManager.create_new_thread(nav_test, THREAD_ID_NAV, (void*)&robot, 25);
	
	gTheThreadManager.create_new_thread(open_CV_image, THREAD_ID_IDENT_IMAGE, (void*)&Camera, 24);

	gTheThreadManager.join_all_threads();
	

	pthread_mutex_destroy ( &movement );


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

void lockMtx(const MUTEX_ID MtxID)
{
	if (MtxID == MUTEX_ID_MOVEMENT)
	{
		pthread_mutex_lock(&movement);
	}
	else
	{
		cerr << "void lockMtx(const MUTEX_ID MtxID) :::: Undefined MtxID" << endl;
	}
}

void unlkMtx(const MUTEX_ID MtxID)
{
	if (MtxID == MUTEX_ID_MOVEMENT)
	{
		pthread_mutex_unlock(&movement);
	}
	else
	{
		cerr << "void unlkMtx(const MUTEX_ID MtxID) :::: Undefined MtxID" << endl;
	}
}
