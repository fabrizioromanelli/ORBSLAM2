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
  ArucoCode(std::string, cv::Point);
  void setCode(std::string);
  void setBboxCenter(cv::Point);
  std::string getCode();
  cv::Point getBboxCenter();

private:
  std::string code;
  cv::Point bBoxCenter;
};

} //namespace ORB_SLAM

#endif // ARUCO_CODE_H
