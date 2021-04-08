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

ArucoCode::ArucoCode(int _code) : code(_code) {}

void ArucoCode::setCode(int _code)
{
  code = _code;
}

void ArucoCode::setBboxCenter(Point2f _bBoxCenter)
{
  bBoxCenter = _bBoxCenter;
}

int ArucoCode::getCode()
{
  return code;
}

Point2f ArucoCode::getBboxCenter()
{
  return bBoxCenter;
}

} //namespace ORB_SLAM
