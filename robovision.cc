/*
 * Known object identification using Open CV
 * Author: Tanvir Amin
 * tanviralamin@gmail.com
 * maamin2@illinois.edu
*/

#include "robovision.hh"
#include "robotest.hh"

<<<<<<< HEAD
int ident( vector<string>& argv1, string argv2) {
=======
int ident( string argv1, string argv2, string argv3, string argv4) {
    // if(argc != 5) {
    //   usage();
    //   return -1;
    // }
>>>>>>> c3185a9649cc26801b5731b1ae6ebd8c3ebdb9ff
  int result = 0;
  try {
    
    Mat img_scene_full = imread(argv2, IMREAD_GRAYSCALE);
    string output_file = "test.jpg";
    float keep_top_fraction = 0.85;

    if(!img_scene_full.data) {
      systemPrint(INFO_SIMPLE, "Error reading images", THREAD_ID_IDENT_IMAGE);
      return -1;
    }

    Mat img_scene;
    // Crop bottom
    // Images taken by mounting the camera on the robot will have some portion
    // of the side of the robot at the bottom. To reduce ambiguity during
    // detection and to speed up feature extraction, we crop it.
    // The fraction of cropping will be different depending on where the camera
    // is mounted on the robot. We find the useful portion of the picture is
    // the top 62.5% when camera mounted on front. When camera mounted on the
    // left side its the top 85% that contains useful information.
    cropBottom(img_scene_full, img_scene, keep_top_fraction);

    // Detect the keypoints and extract descriptors using SURF
    // Surf keypoint detector and descriptor.
    int minHessian = 100;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    Ptr<SURF> detector = SURF::create(
        minHessian, nOctaves, nOctaveLayers, true);


    vector<KeyPoint> keypoints_query, keypoints_scene;
    Mat descriptors_query, descriptors_scene;

    auto sttime = steady_clock::now();
    detector->detectAndCompute(
        img_scene, Mat(), keypoints_scene, descriptors_scene);
      systemPrint(INFO_SIMPLE, "Feature extraction scene image " + to_string((duration <double>(steady_clock::now() - sttime)).count()) + " sec", THREAD_ID_IDENT_IMAGE);

 
    for (unsigned i = 0; i < argv1.size(); ++i){

      Mat img_query = imread(argv1[i], IMREAD_GRAYSCALE);
      if(!img_query.data) {
      systemPrint(INFO_SIMPLE, "Error reading images", THREAD_ID_IDENT_IMAGE);
      return -1;
      }
      sttime = steady_clock::now();
       detector->detectAndCompute(
        img_query, Mat(), keypoints_query, descriptors_query);
      systemPrint(INFO_SIMPLE, "Feature extraction query image " + to_string((duration <double>(steady_clock::now() - sttime)).count()) + " sec", THREAD_ID_IDENT_IMAGE);

      sttime = steady_clock::now();

      // Matching descriptor vectors using Brute Force matcher
      BFMatcher matcher(NORM_L2);
      vector<vector<DMatch>> matches;
      matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);

      vector<DMatch> good_matches;
      for(int i = 0; i < descriptors_query.rows; i++) {
        if (matches[i][0].distance < 0.75 * matches[i][1].distance)
          good_matches.push_back(matches[i][0]);
      }

      // Find the location of the query in the scene
      vector<Point2f> query;
      vector<Point2f> scene;
      for(size_t i = 0; i < good_matches.size(); i++) {
        query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
      }

      vector<Point2f> scene_corners(4);
      bool res = alignPerspective(
        query, scene, img_query, img_scene, scene_corners);
      systemPrint(INFO_SIMPLE, "Matching and alignment " + to_string((duration <double>(steady_clock::now() - sttime)).count()) + " sec", THREAD_ID_IDENT_IMAGE);

      // Write output to file
      Mat img_matches;
      drawMatches(img_query, keypoints_query, img_scene, keypoints_scene,
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

      // Fill the extra area in almost white (Saves ink when printing)
      if (img_query.rows < img_scene.rows) {
        rectangle(img_matches, Point2f(0, img_query.rows),
          Point2f(img_query.cols - 1, img_scene.rows - 1),
          Scalar(255, 240, 240), CV_FILLED);
      } else if (img_scene.rows < img_query.rows) {
        rectangle(img_matches, Point2f(img_query.cols, img_scene.rows),
          Point2f(img_query.cols + img_scene.cols - 1, img_query.rows - 1),
          Scalar(255, 240, 240), CV_FILLED);
<<<<<<< HEAD
      }
      if (res) {
       systemPrint(INFO_NONE, "Object found", THREAD_ID_IDENT_IMAGE);
       drawProjection(img_matches, img_query, scene_corners);
       result = 1;
       cv::imwrite(argv1[i].substr(33).c_str(), img_matches);
       argv1.erase(argv1.begin()+i);
       break;
     } else {
       systemPrint(INFO_NONE, "Object not found", THREAD_ID_IDENT_IMAGE);
     }
      // Write result to a file
     cv::imwrite(output_file, img_matches);
   }

 } catch (cv::Exception& e) {
   systemPrint(INFO_NONE, e.what(), THREAD_ID_IDENT_IMAGE);
   return -1;
 }
 return result;
=======
    }
    if (res) {
	    systemPrint(INFO_NONE, "Object found", THREAD_ID_IDENT_IMAGE);
      drawProjection(img_matches, img_query, scene_corners);
      result = 1;
    } else {
	  systemPrint(INFO_NONE, "Object not found", THREAD_ID_IDENT_IMAGE);
    }
    // Write result to a file
    cv::imwrite(output_file, img_matches);
  } catch (cv::Exception& e) {
	  systemPrint(INFO_NONE, e.what(), THREAD_ID_IDENT_IMAGE);
    return -1;
  }
  return result;
>>>>>>> c3185a9649cc26801b5731b1ae6ebd8c3ebdb9ff
}


void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction) {
  // Crop the lower part of the scene
  cv::Rect crop;
  crop.x = 0;
  crop.y = 0;
  crop.width = img_scene_full.size().width;
  crop.height = img_scene_full.size().height * crop_fraction;
  img_scene = img_scene_full(crop);
}


bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners) {
  Mat H = findHomography(query, scene, RANSAC);
  if (H.rows == 0 && H.cols == 0) {
	systemPrint(INFO_NONE, "Failed rule0: Empty homography", THREAD_ID_IDENT_IMAGE);
    return false;
  }

  vector<Point2f> query_corners(4);
  query_corners[0] = cvPoint(0,0);
  query_corners[1] = cvPoint(img_query.cols, 0);
  query_corners[2] = cvPoint(img_query.cols, img_query.rows);
  query_corners[3] = cvPoint(0, img_query.rows );

  perspectiveTransform(query_corners, scene_corners, H);

  float min_area = 32.0 * 32.0;
  double max_area = img_scene.rows * img_scene.cols;
  float ratio_inside = 0.75;
  float min_angle_sin =  0.173; // Minimum 10 degree angle required

  // Geometric verification heuristics
  // Rule 1: Must be a convex hull.
  // Rule 2: Area can’t be less than 32x32
  // Rule 3: The detected projection can’t have more than 100% area
  // Rule 4: Projection can't contain very small angle < 10 degree
  // Rule 5: More than 75% of the area of the detected projection should have
  // to be within image bounds

  // Rule 1: Must be a convex hull.
  vector<Point2f> sc_vec(4);
  // Generate 4 vectors from the 4 scene corners
  for(int i = 0; i < 4; i++) {
    sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
  }
  vector<float> sc_cross(4);
  // Calculate cross product of pairwise vectors
  for(int i = 0; i < 4; i++) {
    sc_cross[i] = sc_vec[i].cross(sc_vec[(i+1) % 4]);
  }

  // Check for convex hull
  if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0)
      && !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0)) {
	systemPrint(INFO_NONE, "Failed rule1: Not a convex hull", THREAD_ID_IDENT_IMAGE);
    return false;
  }

  // Rule 2: Area can’t be less than 32x32
  // Rule 3: The detected projection can’t have more than 100% area
  float area = (sc_cross[0] + sc_cross[2]) / 2.0;
  if (fabs(area) < min_area) {
	  systemPrint(INFO_NONE, "Failed rule2: Projection too small", THREAD_ID_IDENT_IMAGE);
    return false;
  } else if (fabs(area) > max_area) {
	  systemPrint(INFO_NONE, "Failed rule3: Projection too large", THREAD_ID_IDENT_IMAGE);
    return false;
  }

  // Rule 4: Can't contain very small angle < 10 degree inside projection.
  // Check for angles
  vector<float> sc_norm(4);
  for (int i = 0; i < 4; i++) {
    sc_norm[i] = norm(sc_vec[i]);
  }
  for (int i = 0; i < 4; i++) {
    float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
    if (fabs(sint) < min_angle_sin) {
		systemPrint(INFO_NONE, "Failed rule4: Contains very small angle", THREAD_ID_IDENT_IMAGE);
      return false;
    }
  }

  // Rule 5: More than 75% of the area of the detected projection should
  // have to be within image bounds.
  // Approximate mechanism by determining the bounding rectangle.
  cv::Rect bound = boundingRect(scene_corners);
  cv::Rect scene_rect(0.0, 0.0, img_scene.cols, img_scene.rows);
  cv::Rect isect = bound & scene_rect;
  if (isect.width * isect.height <  ratio_inside * bound.width * bound.height ) {
	  systemPrint(INFO_NONE, "Failed rule5: Large proportion outside scene", THREAD_ID_IDENT_IMAGE);
    return false;
  }
  return true;
}

// Show the projection
void drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners) {
  line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
      scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
      scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
      scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
      scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string type2str(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans + '0');
  return r;
}

void usage() {
  systemPrint(INFO_NONE, " Usage: ./robovision <query-image> <scene-image> <output-image> <crop-bottom>", THREAD_ID_IDENT_IMAGE);
}

