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

namespace ORB_SLAM2
{

class QrCodeTracker
{
public:
  QrCodeTracker();
  // Main tracking function. It is independent of the input sensor.
  void Track(cv::Mat);
  std::string getDecodedData();
  cv::Mat getBoundingBox();
  cv::Mat getRectifiedImage();
  cv::Point getBoundingBoxCenter();
  void display();

protected:
  cv::Mat inputImage;

private:
  cv::QRCodeDetector *qrDecoder;
  cv::Mat bbox, rectifiedImage;
  std::string decodedData;
};

} //namespace ORB_SLAM

#endif // QR_CODE_TRACKER_H
