/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2020 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/

#include "QrCodeTracker.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

QrCodeTracker::QrCodeTracker()
{
  qrDecoder = new QRCodeDetector();
}

void QrCodeTracker::Track(Mat _inputImage)
{
  inputImage = _inputImage;
  decodedData = qrDecoder->detectAndDecode(inputImage, bbox, rectifiedImage);
}

std::string QrCodeTracker::getDecodedData()
{
  if (decodedData.length() > 0)
    return(decodedData);
  else
    return("QR Code not detected");
}

Mat QrCodeTracker::getBoundingBox()
{
  return(bbox);
}

Point QrCodeTracker::getBoundingBoxCenter()
{
  Rect r = boundingRect(bbox);
  Point center(r.x+r.width/2, r.y+r.height/2);
  return(center);
}

Mat QrCodeTracker::getRectifiedImage()
{
  return(rectifiedImage);
}

void QrCodeTracker::display()
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(inputImage, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("Result", inputImage);
  rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
  // imshow("Rectified QRCode", rectifiedImage);
}

} //namespace ORB_SLAM
