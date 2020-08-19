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

QrCodeTracker::QrCodeTracker() : thRect(120, 40, 400, 400)
{
  qrDecoder = new QRCodeDetector();
}

void QrCodeTracker::Track(Mat _inputImage)
{
  inputImage = _inputImage;
  imgWidth   = _inputImage.cols;
  imgHeight  = _inputImage.rows;
  decodedData = qrDecoder->detectAndDecode(inputImage, bbox, rectifiedImage);

  Rect r = boundingRect(bbox);
  Point center(r.x+r.width/2, r.y+r.height/2);
  qrCenter = center;
}

void QrCodeTracker::setThresholds(unsigned int w, unsigned int h)
{
  thRect.x = imgWidth/2 - w/2;
  thRect.y = imgHeight/2 - h/2;
  thRect.width  = w;
  thRect.height = h;
}

bool QrCodeTracker::getDecodedData(std::string & _decodedData)
{
  if (decodedData.length() > 0)
  {
    _decodedData = decodedData;
    return(true);
  }
  else
    return(false);
}

Mat QrCodeTracker::getBoundingBox()
{
  return(bbox);
}

Point QrCodeTracker::getBoundingBoxCenter()
{
  return(qrCenter);
}

Mat QrCodeTracker::getRectifiedImage()
{
  return(rectifiedImage);
}

bool QrCodeTracker::isInsideBbox()
{
  cout << "thRect " << thRect << endl;
  if (qrCenter.inside(thRect))
    return(true);
  else
    return(false);
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
  imshow("Rectified QRCode", rectifiedImage);
}

void QrCodeTracker::addQrCodeToMap(QrCode _newQrCode)
{
  bool addFlag = true;
  for(std::vector<QrCode>::iterator it = qrCodes.begin(); it != qrCodes.end(); ++it) {
    if (!it->getCode().compare(_newQrCode.getCode())) {
      addFlag = false;
      break;
    }
  }

  if (addFlag) {
    qrCodes.push_back(_newQrCode);
    std::cout << "Adding the following QrCode: " << qrCodes[0].getCode() << std::endl;
  } else {
    std::cout << _newQrCode.getCode() << " is already in the list." << std::endl;
  }
}

} //namespace ORB_SLAM
