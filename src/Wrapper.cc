#include "Wrapper.h"
#include <System.h>

void* initSLAM(char *strVocFile, char *strSettingsFile, int sensor, int saveMap) {
  return new ORB_SLAM2::System(string(strVocFile), string(strSettingsFile), static_cast<ORB_SLAM2::System::eSensor>(sensor), false, saveMap > 0);
}

void closeSLAM(void *System) {
  ORB_SLAM2::System *destroy = static_cast<ORB_SLAM2::System *>(System);
  destroy->Shutdown();
  delete destroy;
  return;
}

void runSLAM(void *System, void *imData, void *depthData, int width, int height, double timestamp) {
  ORB_SLAM2::System *slam = static_cast<ORB_SLAM2::System *>(System);

  cv::Mat ir_left(cv::Size(width, height), CV_8UC1, imData, cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(width, height), CV_16UC1, depthData, cv::Mat::AUTO_STEP);

  slam->TrackRGBD(ir_left, depth, timestamp);
}
