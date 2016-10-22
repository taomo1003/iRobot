#include "robotest.hh"
<<<<<<< HEAD
#include "open_CV_image.hh"
#include "robotest.hh"
#include "nav.hh"
=======
#include "nav.hh"
//#include "open_CV_image.hh"
//#include "open_CV_contour.hh"
>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0
#include "safety.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

<<<<<<< HEAD
pthread_mutex_t movement = PTHREAD_MUTEX_INITIALIZER;
=======
pthread_mutex_t safetyMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cameraMutex = PTHREAD_MUTEX_INITIALIZER;

>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0

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
    sensors.push_back(Create::SENSOR_GROUP_1);
    sensors.push_back (Create::SENSOR_BUTTONS);

    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;
	
<<<<<<< HEAD
	gTheThreadManager.create_new_thread(nav_test, THREAD_ID_NAV, (void*)&robot, 25);
	
	gTheThreadManager.create_new_thread(open_CV_image, THREAD_ID_IDENT_IMAGE, (void*)&Camera, 24);

=======
	//gTheThreadManager.create_new_thread(nav_test, THREAD_ID_NAV, (void*)&robot, 25);
	
	//gTheThreadManager.create_new_thread(open_CV_image, THREAD_ID_IDENT_IMAGE, (void*)&Camera, 24);
>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0
	gTheThreadManager.create_new_thread(safety, THREAD_ID_SAFETY, (void*)&robot, 31);

	gTheThreadManager.join_all_threads();
	
<<<<<<< HEAD
=======

>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0
	pthread_mutex_destroy ( &safetyMutex );
	pthread_mutex_destroy ( &cameraMutex );


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
	if (MtxID == MUTEX_ID_SAFETY)
	{
		pthread_mutex_lock(&safetyMutex);
	}
	else if (MtxID == MUTEX_ID_CAMERA)
	{
		pthread_mutex_lock(&cameraMutex);
	}
	else
	{
		cerr << "void lockMtx(const MUTEX_ID MtxID) :::: Undefined MtxID" << endl;
	}
}

void unlkMtx(const MUTEX_ID MtxID)
{
	if (MtxID == MUTEX_ID_SAFETY)
	{
		pthread_mutex_unlock(&safetyMutex);
	}
	else if (MtxID == MUTEX_ID_CAMERA)
	{
		pthread_mutex_unlock(&cameraMutex);
<<<<<<< HEAD
	}
	else
	{
		cerr << "void unlkMtx(const MUTEX_ID MtxID) :::: Undefined MtxID" << endl;
	}
}
=======
	}
	else
	{
		cerr << "void unlkMtx(const MUTEX_ID MtxID) :::: Undefined MtxID" << endl;
	}
}
>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0
