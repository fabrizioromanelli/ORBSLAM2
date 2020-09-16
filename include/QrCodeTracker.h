/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2020 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/


#ifndef QR_CODE_TRACKER_H
#define QR_CODE_TRACKER_H

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "QrCode.h"

namespace ORB_SLAM2
{

class QrCodeTracker
{
public:
  QrCodeTracker();
  // Main tracking function. It is independent of the input sensor.
  void Check(cv::Mat);
  void Track(cv::Mat, cv::Point2d);
  cv::Point Detect(cv::Mat);
  bool getDecodedData(std::string &);
  std::vector<QrCode> * getQrCodeList();
  cv::Mat getBoundingBox();
  cv::Mat getRectifiedImage();
  cv::Point getBoundingBoxCenter();
  void setThresholds(unsigned int, unsigned int);
  bool isInsideBbox();
  void display();
  void addQrCodeToMap(QrCode);
  void loadQrCodeList();
  void saveQrCodeList();
  cv::Point getPositionFromCode(std::string);

protected:
  cv::Mat inputImage;
  unsigned int imgWidth;
  unsigned int imgHeight;

private:
  cv::QRCodeDetector *qrDecoder;
  cv::Mat bbox, rectifiedImage;
  std::string decodedData;
  cv::Point qrCenter;
  cv::Rect thRect;
  std::vector<QrCode> qrCodes;
};

} //namespace ORB_SLAM

#endif // QR_CODE_TRACKER_H
