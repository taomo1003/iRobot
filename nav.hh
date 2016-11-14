#ifndef NAV_H
#define NAV_H

#include "robotest.hh"
#include <math.h>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

struct point
{
point(double nX, double nY):x(nX),y(nY){};

	double x;
	double y;
};

const int ROTATE_SPEED = 200;
const short ROTATE_RADIUS = 400;
const short SPEED = 200;
const short ROBOT_RADIUS = 126;
const short ROTATE_RADIUS_CORRECTION = 45;
const short MIN_MM_BETWEEN_WP = 100;
const bool SHOW_ALL_WP = false;
const chrono::milliseconds MIN_LOCK_TIME = chrono::milliseconds(5);

void* nav_test(void* parms);

chrono::milliseconds sendDriveCommand(Create* robot, const int speed, Create::DriveCommand direction);
chrono::milliseconds sendDriveCommand(Create* robot, const int speed, short radius);
int find_max_wall_signal(Create* robot);
void updatePosition(chrono::system_clock::time_point startTimeForDrive, short currRotateSpeed, short currRotateRadius);
bool recordWaypoint();
void plotWayPoints();

#endif
