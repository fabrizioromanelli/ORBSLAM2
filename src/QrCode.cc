/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2020 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/

#include "QrCode.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

QrCode::QrCode(){}

QrCode::QrCode(std::string _code, cv::Point _bBoxCenter, cv::Point2d _SLAMPosition) : code(_code), bBoxCenter(_bBoxCenter), SLAMPosition(_SLAMPosition) {}

void QrCode::setCode(std::string _code)
{
  code = _code;
}

void QrCode::setBboxCenter(cv::Point _bBoxCenter)
{
  bBoxCenter = _bBoxCenter;
}

void QrCode::setSLAMPosition(cv::Point2d _SLAMPosition)
{
  SLAMPosition = _SLAMPosition;
}

std::string QrCode::getCode()
{
  return code;
}

cv::Point QrCode::getBboxCenter()
{
  return bBoxCenter;
}

cv::Point2d QrCode::getSLAMPosition()
{
  return SLAMPosition;
}

} //namespace ORB_SLAM
