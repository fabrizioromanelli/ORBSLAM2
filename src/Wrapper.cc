#include "Wrapper.h"
#include <System.h>

inline void CalculateRotation( cv::Mat a, double q[] ) {
  double trace = a.at<double>(0, 0) + a.at<double>(1, 1) + a.at<double>(2, 2);
  if( trace > 0 ) {
    double s = 0.5 / sqrt(trace + 1.0);
    q[0] = 0.25 / s;
    q[1] = ( a.at<double>(2, 1) - a.at<double>(1, 2) ) * s;
    q[2] = ( a.at<double>(0, 2) - a.at<double>(2, 0) ) * s;
    q[3] = ( a.at<double>(1, 0) - a.at<double>(0, 1) ) * s;
  } else {
    if ( a.at<double>(0, 0) > a.at<double>(1, 1) && a.at<double>(0, 0) > a.at<double>(2, 2) ) {
      double s = 2.0 * sqrt( 1.0 + a.at<double>(0, 0) - a.at<double>(1, 1) - a.at<double>(2, 2));
      q[0] = (a.at<double>(2, 1) - a.at<double>(1, 2) ) / s;
      q[1] = 0.25 * s;
      q[2] = (a.at<double>(0, 1) + a.at<double>(1, 0) ) / s;
      q[3] = (a.at<double>(0, 2) + a.at<double>(2, 0) ) / s;
    } else if (a.at<double>(1, 1) > a.at<double>(2, 2)) {
      double s = 2.0 * sqrt( 1.0 + a.at<double>(1, 1) - a.at<double>(0, 0) - a.at<double>(2, 2));
      q[0] = (a.at<double>(0, 2) - a.at<double>(2, 0) ) / s;
      q[1] = (a.at<double>(0, 1) + a.at<double>(1, 0) ) / s;
      q[2] = 0.25 * s;
      q[3] = (a.at<double>(1, 2) + a.at<double>(2, 1) ) / s;
    } else {
      double s = 2.0 * sqrt( 1.0 + a.at<double>(2, 2) - a.at<double>(0, 0) - a.at<double>(1, 1) );
      q[0] = (a.at<double>(1, 0) - a.at<double>(0, 1) ) / s;
      q[1] = (a.at<double>(0, 2) + a.at<double>(2, 0) ) / s;
      q[2] = (a.at<double>(1, 2) + a.at<double>(2, 1) ) / s;
      q[3] = 0.25 * s;
    }
  }
}

/**
 * This function initialize the ORBSLAM2 algorithm
 * 
 * @param  strVocFile Represents the vocabulary file location
 * @param  strSettingsFile Represents the configuration of the camera & algorithm file location
 * @param  sensor Represents the sensor type (0 for Monocular, 1 for Stereo, 2 for RGBD)
 * @param  saveMap Represents the a boolean value (0 or 1) to tell the system to save a map at the end of the algorithm or not
 * @return a pointer to a class of type System
 */
void* initSLAM(char *strVocFile, char *strSettingsFile, int sensor, int saveMap) {
  return new ORB_SLAM2::System(string(strVocFile), string(strSettingsFile), static_cast<ORB_SLAM2::System::eSensor>(sensor), false, saveMap > 0);
}

/**
 * This function closes the ORBSLAM2 algorithm and frees all the dynamic memory allocation
 * 
 * @param  System Represents the pointer to a class of type System
 * @return none
 */
void closeSLAM(void *System) {
  ORB_SLAM2::System *destroy = static_cast<ORB_SLAM2::System *>(System);
  destroy->Shutdown();
  delete destroy;
  return;
}

/**
 * This function runs the ORBSLAM2 algorithm (must be previously initialized)
 * 
 * @param  System Represents the pointer to a class of type System
 * @param  imData Represents the pointer to an infrared frame
 * @param  depthData Represents the pointer to a depth frame
 * @param  width Represents the width of the frames
 * @param  height Represents the height of the frames
 * @param  timestamp Represents the timestamp of the frame
 * @return an array of doubles with [x, y, z] position and a quaternion [qw, qx, qy, qz] for the orientation
 */
double * runSLAM(void *System, void *imData, void *depthData, int width, int height, double timestamp) {
  double _cameraPose[7], _quaternion[4];
  ORB_SLAM2::System *slam = static_cast<ORB_SLAM2::System *>(System);

  cv::Mat ir_left(cv::Size(width, height), CV_8UC1, imData, cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(width, height), CV_16UC1, depthData, cv::Mat::AUTO_STEP);

  cv::Mat cameraPose = slam->TrackRGBD(ir_left, depth, timestamp);
  CalculateRotation(cameraPose, _quaternion);
}
