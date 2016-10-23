#ifndef ROBOTEST_H
#define ROBOTEST_H

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

using namespace iRobot;
using namespace LibSerial;
using namespace std;

const int WALL_SENSOR_WALL = 200;
const int ACCEPTED_OFF_MAX = 5;

enum INFO_LEVEL:int
{
	INFO_NONE,
	INFO_SIMPLE,
	INFO_ALL
};

const INFO_LEVEL gPrintDebug = INFO_ALL;

enum THREAD_ID:int
{
	THREAD_ID_MAIN,
	THREAD_ID_NAV,
	THREAD_ID_IDENT_IMAGE,
	THREAD_ID_IDENT_CONTOUR,
	THREAD_ID_SAFETY,
	N_THREADS
};

enum MUTEX_ID:int
{
	MUTEX_ID_SAFETY,
	MUTEX_ID_CAMERA,
	MUTEX_ID_PRINT,
	MUTEX_ID_SERIAL
};

void systemPrint(const INFO_LEVEL lvl,  const string s, const THREAD_ID id);

class thread_manager
{
private:
	pthread_t callThd[N_THREADS];
	sched_param sch_params[N_THREADS];
	bool madeThreads[N_THREADS];
protected:
	void create_thread(void*(*func)(void*), THREAD_ID Id, void* params)
	{
		systemPrint(INFO_ALL, "Creating thread " + Id, THREAD_ID_MAIN);
		pthread_create(&callThd[Id], NULL, func, params);
	}
	void set_thread_priority(THREAD_ID Id, const int priority)
	{
		sch_params[Id].sched_priority = priority;	
		if (pthread_setschedparam(callThd[Id], SCHED_FIFO, &sch_params[Id]))
		{
			systemPrint(INFO_NONE, "Failed to Set Thread Priority For Thread " + to_string(Id), THREAD_ID_MAIN);
		}
	}
	void join_thread(THREAD_ID Id)
	{
		systemPrint(INFO_ALL, "Joining thread " + Id, THREAD_ID_MAIN);
		pthread_join(callThd[Id], NULL);
	}
public:
	thread_manager()
	{
		for (int i = 0; i < N_THREADS; ++i)
		{
			madeThreads[i] = false;
		}
	}
	void create_new_thread(void*(*func)(void*), THREAD_ID Id, void* params, int priority)
	{
		create_thread(func, Id, params);
		set_thread_priority(Id, priority);
		madeThreads[Id] = true;
	}
	void join_all_threads()
	{
		for (int i = 0; i < N_THREADS; ++i)
		{
			if (madeThreads[i])
				join_thread((THREAD_ID)i);
		}
	}
};

void lockMtx(const MUTEX_ID MtxID, const THREAD_ID Id);
void unlkMtx(const MUTEX_ID MtxID, const THREAD_ID Id);


#endif
