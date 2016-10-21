#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

using std::chrono::duration;
using std::chrono::steady_clock;
using std::cout;
using std::endl;
using std::string;
using std::vector;

bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
void drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners);
string type2str(int type);
void usage();

int ident( string argv1, string argv2, string argv3, string argv4);