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

void QrCodeTracker::Check(Mat _inputImage)
{
  inputImage = _inputImage;
  imgWidth   = _inputImage.cols;
  imgHeight  = _inputImage.rows;

  decodedData = qrDecoder->detectAndDecode(inputImage, bbox, rectifiedImage);

  try
  {
    Rect r = boundingRect(bbox);
    Point center(r.x+r.width/2, r.y+r.height/2);
    qrCenter = center;
  }
  catch(const std::exception& e){}
}

void QrCodeTracker::Track(cv::Mat _inputImage, cv::Point2d _SLAMPosition)
{
  std::string decodedData;

  this->Check(_inputImage);

  if (this->getDecodedData(decodedData) && this->isInsideBbox()) {
    QrCode newQrCode(decodedData, this->getBoundingBoxCenter(), _SLAMPosition);
    this->addQrCodeToMap(newQrCode);
  }
}

Point QrCodeTracker::Detect(cv::Mat _inputImage)
{
  Point temp;
  std::string decodedData;

  this->Check(_inputImage);

  if (this->getDecodedData(decodedData)) {
    temp.x = this->getBoundingBoxCenter().x - imgWidth/2;
    temp.y = this->getBoundingBoxCenter().y - imgHeight/2;
  } else {
    // Fixing a high value for both X and Y in order to
    // always return an initialized value.
    temp.x = 9999;
    temp.y = 9999;
  }
  return(temp);
}

istream& operator >> (istream &fi, cv::Point &p)
{
   char char1, char2, char3;
   fi >> char1 >> p.x >>char2 >> p.y >> char3;
   if (char1=='[' &&  char2==',' && char3==']')
    return fi;
}

istream& operator >> (istream &fi, cv::Point2d &p)
{
   char char1, char2, char3;
   fi >> char1 >> p.x >>char2 >> p.y >> char3;
   if (char1=='[' &&  char2==',' && char3==']')
    return fi;
}

void QrCodeTracker::loadQrCodeList()
{
  std::string empty, code;
  cv::Point bBoxCenter;
  cv::Point2d SLAMPosition;
  std::ifstream in("QRCodes.txt");

  int lineNum = 0;
  std::string line;
  while (std::getline(in, line))
  {
    std::istringstream iss(line);
    if (lineNum == 1)
      code = line;
    if (lineNum == 2)
      iss >> bBoxCenter;
    if (lineNum == 3) {
      iss >> SLAMPosition;
      lineNum = 0;
      QrCode newQrCode(code, bBoxCenter, SLAMPosition);
      this->addQrCodeToMap(newQrCode);
      continue;
    }
    lineNum++;
  }
}

void QrCodeTracker::saveQrCodeList()
{
  std::ofstream out("QRCodes.txt");

  // Loop over all the detected QRCodes
  int i = 1;
  for(std::vector<QrCode>::iterator it = qrCodes.begin(); it != qrCodes.end(); ++it, ++i) {
    out << "#### Detected code " << i << " ####" << endl;
    out << it->getCode() << endl;
    out << it->getBboxCenter() << endl;
    out << it->getSLAMPosition() << endl;
  }

  out.close();
}

std::vector<QrCode> * QrCodeTracker::getQrCodeList()
{
  return &qrCodes;
}

cv::Point QrCodeTracker::getPositionFromCode(std::string _inputCode)
{
  for(std::vector<QrCode>::iterator it = qrCodes.begin(); it != qrCodes.end(); ++it) {
    if (!it->getCode().compare(_inputCode)) {
      return (it->getSLAMPosition());
    }
  }

  return (cv::Point(-1, -1));
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
  // rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
  // imshow("Rectified QRCode", rectifiedImage);
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
    std::cout << "Adding the following QrCode: " << _newQrCode.getCode() << std::endl;
  } else {
    std::cout << _newQrCode.getCode() << " is already in the list." << std::endl;
  }
}

} //namespace ORB_SLAM
