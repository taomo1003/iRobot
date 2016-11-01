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
#include <semaphore.h>

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

const INFO_LEVEL gPrintDebug = INFO_SIMPLE;

enum THREAD_ID:int
{
	THREAD_ID_MAIN,
	THREAD_ID_NAV,
	THREAD_ID_IDENT_IMAGE,
	THREAD_ID_SAFETY,
	THREAD_ID_EXTERNAL,
	N_THREADS
};

//static string threadNames[N_THREADS];

enum MUTEX_ID:int
{
	MUTEX_ID_SAFETY,
	MUTEX_ID_CAMERA,
	MUTEX_ID_PRINT,
	MUTEX_ID_SERIAL
};

void systemPrint(const INFO_LEVEL lvl,  const string s, const THREAD_ID id);
string threadIdToString(const THREAD_ID Id);

class thread_manager
{
private:
	pthread_t callThd[N_THREADS];
	sched_param sch_params[N_THREADS];
	bool madeThreads[N_THREADS];
protected:
	void create_thread(void*(*func)(void*), THREAD_ID Id, void* params,int p)
	{
		systemPrint(INFO_SIMPLE, "Creating thread " + threadIdToString(Id), THREAD_ID_MAIN);
		
		pthread_attr_t tattr;
		sched_param param;

		/* initialized with default attributes */
		pthread_attr_init (&tattr);

		/* safe to get existing scheduling param */
		pthread_attr_getschedparam (&tattr, &param);

		/* set the priority; others are unchanged */
		param.sched_priority = p;

		/* setting the new scheduling param */
		pthread_attr_setschedparam (&tattr, &param);
		pthread_create(&callThd[Id], &tattr, func, params);
	}
	void set_thread_priority(THREAD_ID Id, const int priority)
	{
		sch_params[Id].sched_priority = priority;	
		int status = pthread_setschedparam(callThd[Id], SCHED_FIFO, &sch_params[Id]);
		if (status != 0)
		{
			systemPrint(INFO_NONE, "Failed to Set Thread Priority For Thread " + threadIdToString(Id), THREAD_ID_MAIN);
		}
	}
	void join_thread(THREAD_ID Id)
	{
		pthread_join(callThd[Id], NULL);
		systemPrint(INFO_SIMPLE, "Joining thread " + threadIdToString(Id), THREAD_ID_MAIN);
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
		create_thread(func, Id, params, priority);
		//set_thread_priority(Id, priority);
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

bool stop_running_thread(const THREAD_ID Id);

void lockMtx(const MUTEX_ID MtxID, const THREAD_ID Id);
void unlkMtx(const MUTEX_ID MtxID, const THREAD_ID Id);

#endif