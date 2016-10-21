
using namespace iRobot;
using namespace LibSerial;
using namespace std;

const int ROTATE_SPEED = 200;
const short ROTATE_RADIUS = 400;

void* nav_test(void* parms);

void sendDriveCommand(Create* robot, const int speed, Create::DriveCommand direction);
void sendDriveCommand(Create* robot, const int speed, short radius);
int find_max_wall_signal(Create* robot);