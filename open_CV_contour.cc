using namespace iRobot;
using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void* open_CV_contour(void* params)


void* open_CV_contour(void* params)
{	
	vector<Point2f> waypoints; // Create a list of points.
	// Insert some points.
	Point2f apoint(10,10); // Create point (10, 10).
	waypoints.push_back(apoint);
	lineWidthaypoints.push_back(Point2f(100, 200)); // Insert point (100, 200) directly.
	// Print those points.
	for (auto point : waypoints)
		std::cout << point << std::endl;

	Mat img_output(1200, 1600, CV_8UC3, Scalar(255, 255, 255));
	// Plot the waypoints using blue color.
	Scalar lineColor(255, 0, 0);
	int lineWidth = 1;
	int radius = 3;
	for (int i = 0; i < waypoints.size() - 1; i++) {
		line(img_output, waypoints[i], waypoints[i + 1],
		lineColor, lineWidth, CV_AA);
		circle(img_output, waypoints[i], radius,
		lineColor, CV_FILLED, CV_AA);
	}
	// Draw the bounding rectangle using orange color
	Rect bound = boundingRect(waypoints);
	rectangle(img_output, bound, Scalar(0, 165, 255));
	// Finally store it as a png file
	imwrite("irobot_plot.png", img_output);
}