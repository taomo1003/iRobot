#include "robotest.hh"
#include "nav.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_t callThd[N_THREADS];

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
	
	movement = new sem_t;
	sem_init(movement, true, 1);
	//thread threads[N_THREADS];
	
	pthread_create(&callThd[THREAD_ID_NAV], NULL, nav_test, (void*)&robot);
	// 0-31 priorities
	sched_param sch_params;
	
	sch_params.sched_priority = 25;
	
	if (pthread_setschedparam(callThd[THREAD_ID_NAV], SCHED_FIFO, &sch_params))
	{
		cerr << "Failed to set thread priority." << endl;
	}
	
	for(int i=0; i<N_THREADS;i++)
	{
		pthread_join(callThd[i],NULL);
	}
	sem_destroy( movement );
	delete movement;
	
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
