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

void QrCodeTracker::Track(Mat inputImage)
{
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

Mat QrCodeTracker::getRectifiedImage()
{
  return(rectifiedImage);
}


} //namespace ORB_SLAM
