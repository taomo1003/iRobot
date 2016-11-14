#include "nav.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace cv;


vector<Point2f> wayPoints;

Point2f currPoint;
float angle;

void* nav_test(void* parms)
{	
	Create* robot = (Create*)parms;
	if(robot==nullptr)
	{
		systemPrint(INFO_NONE, "Robot Not Defined", THREAD_ID_NAV);
		
		return nullptr;
	}
	short wallSignal = 0;
	short prevWallSignal =0;
	short maxWallSignal = 0;
	short wallSignalFail = 0;	
	
	Point2f p (0,0);
	wayPoints.push_back(p);	
	
	auto startTimeForDrive = std::chrono::system_clock::now();
	
	bool loop = stop_running_thread(THREAD_ID_NAV);
	while(!loop)
	{
		auto start_time = std::chrono::system_clock::now();
		auto deadline = start_time + std::chrono::milliseconds(100);
		
		sendDriveCommand(robot,SPEED, Create::DRIVE_STRAIGHT);
		
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
		loop = robot->bumpRight()|robot->bumpLeft();
		loop |= stop_running_thread(THREAD_ID_NAV);
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
		
		this_thread::sleep_until(deadline);
	}
	
	systemPrint(INFO_SIMPLE, "Bump", THREAD_ID_NAV);
  
    sendDriveCommand(robot,-SPEED, Create::DRIVE_STRAIGHT);
    this_thread::sleep_for(chrono::milliseconds(30000 / SPEED));
    sendDriveCommand(robot,0, Create::DRIVE_STRAIGHT);

    maxWallSignal = find_max_wall_signal(robot);

	currPoint.x = 0;
	currPoint.y = 0;
	angle = 0;
	
	short currRotateSpeed = 0;
	short currRotateRadius = 0;
	
	loop = stop_running_thread(THREAD_ID_NAV);
	
	bool wayPointRecorded = false;
	while (!loop)
    {
		auto start_time = std::chrono::system_clock::now();
		auto deadline = start_time + std::chrono::milliseconds(100);
		
	  loop = stop_running_thread(THREAD_ID_NAV);
	  lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
      wallSignal = robot->wallSignal();
	  bool bBumpLeft = robot->bumpLeft();
	  bool bBumpRight = robot->bumpRight();
	  unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	  
	  systemPrint(INFO_ALL, to_string(wallSignal), THREAD_ID_NAV);
	  
	  
	  if( bBumpLeft || bBumpRight )
	  {
		  wayPointRecorded = false;
		  systemPrint(INFO_SIMPLE, "Bump", THREAD_ID_NAV);
		  
		  updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);
				
		  systemPrint(INFO_SIMPLE, "Starting Backing Up", THREAD_ID_NAV);
		  
		  recordWaypoint();
		  
		  sendDriveCommand(robot,-SPEED, Create::DRIVE_STRAIGHT);
		  startTimeForDrive = std::chrono::system_clock::now();
		  currRotateRadius = -1;
		  currRotateSpeed = -SPEED;
		  this_thread::sleep_for(chrono::milliseconds(30000 / SPEED));
		  updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);
		  sendDriveCommand(robot,0, Create::DRIVE_STRAIGHT);
	  
		  maxWallSignal = find_max_wall_signal(robot);
		  
		  sendDriveCommand(robot,SPEED, Create::DRIVE_STRAIGHT);
		  
		  startTimeForDrive = std::chrono::system_clock::now();
		  currRotateRadius = -1;
		  currRotateSpeed = SPEED;
	  }
	  else if(wallSignal <= maxWallSignal - ACCEPTED_OFF_MAX)
	  {
		sendDriveCommand(robot,ROTATE_SPEED, -ROTATE_RADIUS);
		
		if (currRotateRadius != -ROTATE_RADIUS || currRotateSpeed != ROTATE_SPEED)
		{
			updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);
		
			startTimeForDrive = std::chrono::system_clock::now();
			currRotateRadius = -ROTATE_RADIUS;
			currRotateSpeed = ROTATE_SPEED;
			
			wayPointRecorded = false;
			
			systemPrint(INFO_SIMPLE, "Starting C Arc", THREAD_ID_NAV);
		}
	  }
	  else if(wallSignal >= maxWallSignal + ACCEPTED_OFF_MAX)
	  {
		sendDriveCommand(robot,ROTATE_SPEED, ROTATE_RADIUS);
		
		if (currRotateRadius != ROTATE_RADIUS || currRotateSpeed != ROTATE_SPEED)
		{
			updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);

			startTimeForDrive = std::chrono::system_clock::now();
			currRotateRadius = ROTATE_RADIUS;
			currRotateSpeed = ROTATE_SPEED;
			
			wayPointRecorded = false;
			
			systemPrint(INFO_SIMPLE, "Starting CC Arc", THREAD_ID_NAV);
		}
	  }
	  else
	  {
		 sendDriveCommand(robot,SPEED, Create::DRIVE_STRAIGHT);	
		 
		if (currRotateRadius != -1 || currRotateSpeed != SPEED)
		{	
			updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);

			startTimeForDrive = std::chrono::system_clock::now();
			currRotateRadius = -1;
			currRotateSpeed = SPEED;
			
			wayPointRecorded = false;
			
			systemPrint(INFO_SIMPLE, "Starting Straight", THREAD_ID_NAV);
		}
	  }
	  
	  if (wallSignal < ACCEPTED_OFF_MAX)
	  {
		  if (!wayPointRecorded)
		  {
			  recordWaypoint();
			  wayPointRecorded = true;
		  }
	  }
	  this_thread::sleep_until(deadline);
	  updatePosition(startTimeForDrive, currRotateSpeed, currRotateRadius);
	  startTimeForDrive = std::chrono::system_clock::now();
	  recordWaypoint();
	  prevWallSignal = wallSignal;
	}
	
	sendDriveCommand(robot,0, ROTATE_RADIUS);
	plotWayPoints();
	return nullptr;
  }

void sendDriveCommand(Create* robot, const int speed, Create::DriveCommand direction)
{
	lockMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	robot->sendDriveCommand(speed, direction);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
}

void sendDriveCommand(Create* robot, const int speed, short radius)
{	
	lockMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	robot->sendDriveCommand(speed, radius);
	unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_SAFETY, THREAD_ID_NAV);
	unlkMtx(MUTEX_ID_CAMERA, THREAD_ID_NAV);
}

int find_max_wall_signal(Create* robot)
{
	short wallSignal;
	short maxWallSignal = -1;
	short prevWallSignal = 0;
	bool inc = false;
	
	bool killThread = stop_running_thread(THREAD_ID_NAV);
	
	systemPrint(INFO_SIMPLE, "Spinning to Parallel", THREAD_ID_NAV);
	  
	  for (int i = 0; i < 40000 / ROTATE_SPEED; ++i)
	  {
		killThread = stop_running_thread(THREAD_ID_NAV);
		
		if (killThread)
		{
			return 1;
		}
		
	    sendDriveCommand(robot,ROTATE_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		auto startDriveTime = std::chrono::system_clock::now();
		this_thread::sleep_for(chrono::milliseconds(15));
		updatePosition(startDriveTime, ROTATE_SPEED, 0);
		prevWallSignal = wallSignal;
		
		lockMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);
		wallSignal = robot->wallSignal();
		unlkMtx(MUTEX_ID_SERIAL, THREAD_ID_NAV);

		
		//cout << wallSignal << endl;
		if (wallSignal - prevWallSignal >= 4)
		{
			inc = true;
		}
		if (inc && wallSignal < prevWallSignal && wallSignal<40)
		{
			maxWallSignal = prevWallSignal;
			systemPrint(INFO_SIMPLE, "Found local maximum: " + to_string(maxWallSignal), THREAD_ID_NAV);
			break;
		}
	  }
	  systemPrint(INFO_SIMPLE, "Done", THREAD_ID_NAV);
	  sendDriveCommand(robot,0, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	  
	  return maxWallSignal;
}
void updatePosition(chrono::system_clock::time_point startTimeForDrive, short currRotateSpeed, short currRotateRadius)
{
	float driveTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - startTimeForDrive).count() / 1000.0f;
	
	float angleOfRotation =  0;
	const double PI = acos(0.0)*2;
	
	if (currRotateRadius != 0)
	{
		angleOfRotation = (currRotateSpeed * driveTime / currRotateRadius) /PI * 180;
	}
	else
	{
		angleOfRotation = (currRotateSpeed * driveTime / ROBOT_RADIUS) /PI * 180;
	}
	
	if (currRotateRadius == -1)
	{
		angleOfRotation = 0;
	}
	
	float oldAngle = angle;
	angle += angleOfRotation;
	
	while (angle >= 180)
	{
		angle -= 360;
	}
	while (angle < -180)
	{
		angle += 360;
	}
	
	systemPrint(INFO_SIMPLE, (string)" driveTime: " + to_string(driveTime), THREAD_ID_NAV);
	systemPrint(INFO_SIMPLE, (string)" Angle: " + to_string(angle), THREAD_ID_NAV);
	
	float delta_x = (-cos((oldAngle + 90)*(PI/180)) + cos((angle + 90)*(PI/180)))*currRotateRadius;
	float delta_y = (-sin((oldAngle + 90)*(PI/180)) + sin((angle + 90)*(PI/180)))*currRotateRadius;
	
	if (currRotateRadius == -1)
	{
		delta_x = currRotateSpeed * driveTime * cos((angle)*(PI/180));
		delta_y = currRotateSpeed * driveTime * sin((angle)*(PI/180));
	}
	
	currPoint.x += delta_x;
	currPoint.y += delta_y;
}
void recordWaypoint()
{
	float delta_x = currPoint.x-wayPoints.back().x;
	float delta_y = currPoint.y-wayPoints.back().y;
	
	systemPrint(INFO_SIMPLE, (string)"Created Waypoint " + to_string(wayPoints.size()) + (string)" delta(x): " + to_string(delta_x) + (string)" delta(y): " + to_string(delta_y), THREAD_ID_NAV);
	systemPrint(INFO_SIMPLE, (string)"                 Angle: " + to_string(angle), THREAD_ID_NAV);

	
	wayPoints.push_back(currPoint);
}
void plotWayPoints()
{
	const int MARGIN = 10;
	
	Point2f maxNegWaypoint(0,0);
	float maxSize = 0;
	
	for (int i = 0; i  < wayPoints.size(); ++i)
	{
		wayPoints[i].y = -wayPoints[i].y;
	}
	for (int i = 0; i  < wayPoints.size(); ++i)
	{
		if (maxNegWaypoint.x > wayPoints[i].x)
		{
			maxNegWaypoint.x = wayPoints[i].x;
		}
		if (maxNegWaypoint.y > wayPoints[i].y)
		{
			maxNegWaypoint.y = wayPoints[i].y;
		}
	}
	for (int i = 0; i  < wayPoints.size(); ++i)
	{
		wayPoints[i].x -= maxNegWaypoint.x - MARGIN;
		wayPoints[i].y -= maxNegWaypoint.y - MARGIN;
	}
	
	for (int i = 0; i  < wayPoints.size(); ++i)
	{
		if (maxSize < abs(wayPoints[i].x))
		{
			maxSize = abs(wayPoints[i].x);
		}
		if (maxSize < abs(wayPoints[i].y))
		{
			maxSize = abs(wayPoints[i].y);
		}
	}
	
	for (int i = 0; i  < wayPoints.size(); ++i)
	{
		wayPoints[i].x *= (1200-2*MARGIN)/maxSize;
		wayPoints[i].y *= (1200-2*MARGIN)/maxSize;
	}
	
	Rect sizeAfterScale = boundingRect(wayPoints);
	
	systemPrint(INFO_SIMPLE, "Ploting Waypoints", THREAD_ID_NAV);
	// Create a drawing context, and use white background.
	Mat img_output(sizeAfterScale.height + 2*MARGIN, sizeAfterScale.width + 2*MARGIN, CV_8UC3, Scalar(255, 255, 255));
	// Plot the waypoints using blue color.
	Scalar lineColor(255, 0, 0);
	int lineWidth = 1;
	int radius = 3;
	for (int i = 0; i < wayPoints.size() - 1; i++) {
	arrowedLine(img_output, wayPoints[i], wayPoints[i + 1],
	lineColor, lineWidth, CV_AA);
	circle(img_output, wayPoints[i], radius,
	lineColor, CV_FILLED, CV_AA);
	systemPrint(INFO_SIMPLE, (string)"Waypoint " + to_string(i) + (string)": (" + to_string(wayPoints[i].x) +(string)", " + to_string(wayPoints[i].y) + (string)")", THREAD_ID_NAV);
	}
	// Draw the bounding rectangle using orange color
	Rect bound = boundingRect(wayPoints);
	rectangle(img_output, bound, Scalar(0, 165, 255));
	// Finally store it as a png file
	imwrite("irobot_plot.png", img_output);
	systemPrint(INFO_SIMPLE, "Waypoints plotted", THREAD_ID_NAV);
}