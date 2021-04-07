/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2021 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/

#include "ArucoCode.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

ArucoCode::ArucoCode(){}

ArucoCode::ArucoCode(std::string _code, cv::Point _bBoxCenter) : code(_code), bBoxCenter(_bBoxCenter) {}

void ArucoCode::setCode(std::string _code)
{
  code = _code;
}

void ArucoCode::setBboxCenter(cv::Point _bBoxCenter)
{
  bBoxCenter = _bBoxCenter;
}

std::string ArucoCode::getCode()
{
  return code;
}

cv::Point ArucoCode::getBboxCenter()
{
  return bBoxCenter;
}

} //namespace ORB_SLAM
