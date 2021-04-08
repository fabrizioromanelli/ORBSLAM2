/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2021 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/


#ifndef ARUCO_CODE_H
#define ARUCO_CODE_H

#include <opencv2/aruco.hpp>
#include <iostream>

namespace ORB_SLAM2
{

class ArucoCode
{
public:
  ArucoCode();
  ArucoCode(int);
  void setCode(int);
  void setBboxCenter(cv::Point2f);
  int getCode();
  cv::Point2f getBboxCenter();

private:
  int code;
  cv::Point2f bBoxCenter;
};

} //namespace ORB_SLAM

#endif // ARUCO_CODE_H
