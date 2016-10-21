#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

const int WALL_SENSOR_WALL = 200;
const int ACCEPTED_OFF_MAX = 5;

sem_t *movement;

enum THREAD_ID:int
{
	THREAD_ID_NAV,
	N_THREADS
};