/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2021 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/


#ifndef ARUCO_CODE_SCANNER_H
#define ARUCO_CODE_SCANNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "ArucoCode.h"

namespace ORB_SLAM2
{

class ArucoCodeScanner
{
public:
  ArucoCodeScanner();
  // Main scanner function. It is independent of the input sensor.
  void Scan(cv::Mat);
  cv::Point Detect(cv::Mat);
  cv::Mat getBoundingBox();
  cv::Point getBoundingBoxCenter();
  void display();
  void loadArucoCodeList();

protected:
  cv::Mat inputImage;

private:
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Point arucoCenter;
  std::vector<ArucoCode> arucoCodes;
};

} //namespace ORB_SLAM

#endif // ARUCO_CODE_SCANNER_H
