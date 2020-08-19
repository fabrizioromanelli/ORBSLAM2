/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2020 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/


#ifndef QR_CODE_H
#define QR_CODE_H

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace ORB_SLAM2
{

class QrCode
{
public:
  QrCode();
  QrCode(std::string, cv::Point, cv::Point2d);
  void setCode(std::string);
  void setBboxCenter(cv::Point);
  void setSLAMPosition(cv::Point2d);
  std::string getCode();
  cv::Point getBboxCenter();
  cv::Point2d getSLAMPosition();

private:
  std::string code;
  cv::Point bBoxCenter;
  cv::Point2d SLAMPosition;
};

} //namespace ORB_SLAM

#endif // QR_CODE_H
