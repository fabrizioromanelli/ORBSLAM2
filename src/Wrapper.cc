#include "Wrapper.h"
#include <System.h>

inline void computeQuaternion( cv::Mat a, float q[] ) {
  float trace = a.at<float>(0, 0) + a.at<float>(1, 1) + a.at<float>(2, 2);
  if( trace > 0 ) {
    float s = 0.5 / sqrt(trace + 1.0);
    q[0] = 0.25 / s;
    q[1] = ( a.at<float>(2, 1) - a.at<float>(1, 2) ) * s;
    q[2] = ( a.at<float>(0, 2) - a.at<float>(2, 0) ) * s;
    q[3] = ( a.at<float>(1, 0) - a.at<float>(0, 1) ) * s;
  } else {
    if ( a.at<float>(0, 0) > a.at<float>(1, 1) && a.at<float>(0, 0) > a.at<float>(2, 2) ) {
      float s = 2.0 * sqrt( 1.0 + a.at<float>(0, 0) - a.at<float>(1, 1) - a.at<float>(2, 2));
      q[0] = (a.at<float>(2, 1) - a.at<float>(1, 2) ) / s;
      q[1] = 0.25 * s;
      q[2] = (a.at<float>(0, 1) + a.at<float>(1, 0) ) / s;
      q[3] = (a.at<float>(0, 2) + a.at<float>(2, 0) ) / s;
    } else if (a.at<float>(1, 1) > a.at<float>(2, 2)) {
      float s = 2.0 * sqrt( 1.0 + a.at<float>(1, 1) - a.at<float>(0, 0) - a.at<float>(2, 2));
      q[0] = (a.at<float>(0, 2) - a.at<float>(2, 0) ) / s;
      q[1] = (a.at<float>(0, 1) + a.at<float>(1, 0) ) / s;
      q[2] = 0.25 * s;
      q[3] = (a.at<float>(1, 2) + a.at<float>(2, 1) ) / s;
    } else {
      float s = 2.0 * sqrt( 1.0 + a.at<float>(2, 2) - a.at<float>(0, 0) - a.at<float>(1, 1) );
      q[0] = (a.at<float>(1, 0) - a.at<float>(0, 1) ) / s;
      q[1] = (a.at<float>(0, 2) + a.at<float>(2, 0) ) / s;
      q[2] = (a.at<float>(1, 2) + a.at<float>(2, 1) ) / s;
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
 * @return an array of floats with [x, y, z] position and a quaternion [qw, qx, qy, qz] for the orientation (the values are concat)
 */
float * runSLAM(void *System, void *imData, void *depthData, int width, int height, double timestamp) {
  float _cameraPose[7], _quaternion[4];
  ORB_SLAM2::System *slam = static_cast<ORB_SLAM2::System *>(System);

  cv::Mat ir_left(cv::Size(width, height), CV_8UC1, imData, cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(width, height), CV_16UC1, depthData, cv::Mat::AUTO_STEP);

  cv::Mat cameraPose = slam->TrackRGBD(ir_left, depth, timestamp);

  if (!cameraPose.empty()) {
    computeQuaternion(cameraPose, _quaternion);

    // Filling up the output
    _cameraPose[0] = cameraPose.at<float>(0, 3);
    _cameraPose[1] = cameraPose.at<float>(1, 3);
    _cameraPose[2] = cameraPose.at<float>(2, 3);
    _cameraPose[3] = _quaternion[0];
    _cameraPose[4] = _quaternion[1];
    _cameraPose[5] = _quaternion[2];
    _cameraPose[6] = _quaternion[3];
  } else {
    _cameraPose[0] = _cameraPose[1] = _cameraPose[2] = _cameraPose[3] = _cameraPose[4] = _cameraPose[5] = _cameraPose[6] = 0.0;
  }

  return(_cameraPose);
}

/**
 * This function check the status of the ORBSLAM2 algorithm
 * 
 * @param  System Represents the pointer to a class of type System
 * @return an array of ints with number of loop closures and an int with -1 (system not ready), 0 (no images yet), 1 (not initialized), 2 (ok), 3 (track lost)
 */
int * statusSLAM(void *System) {
  int _status[2];
  ORB_SLAM2::System *slam = static_cast<ORB_SLAM2::System *>(System);

  // enum eTrackingState{
  //   SYSTEM_NOT_READY=-1,
  //   NO_IMAGES_YET=0,
  //   NOT_INITIALIZED=1,
  //   OK=2,
  //   LOST=3
  // };
  _status[1] = slam->GetTrackingState();

  return(_status);
}