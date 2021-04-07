/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2021 Fabrizio Romanelli <fabrizio dot romanelli at gmail dot com> (University of Rome Tor Vergata)
*/

#include "ArucoCodeScanner.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

ArucoCodeScanner::ArucoCodeScanner()
{
  parameters = cv::aruco::DetectorParameters::create();
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

// Scan for any Aruco code in the image
void ArucoCodeScanner::Scan(Mat _inputImage)
{
  inputImage = _inputImage;

  cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  // try
  // {
  //   Rect r = boundingRect(bbox);
  //   Point center(r.x+r.width/2, r.y+r.height/2);
  //   arucoCenter = center;
  // }
  // catch(const std::exception& e){}
}

// Scan for any Aruco code in the image and returns true if the code is
// in the list of "good" ones.
Point ArucoCodeScanner::Detect(cv::Mat _inputImage)
{
  Point temp;
  std::string decodedData;

  this->Scan(_inputImage);

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

void ArucoCodeScanner::loadArucoCodeList()
{
  std::string empty, code;
  cv::Point bBoxCenter;
  std::ifstream in("arucoCodes.dat");

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
      ArucoCode newQrCode(code, bBoxCenter);
      this->addQrCodeToMap(newQrCode);
      continue;
    }
    lineNum++;
  }
}

Mat QrCodeTracker::getBoundingBox()
{
  return(bbox);
}

Point QrCodeTracker::getBoundingBoxCenter()
{
  return(qrCenter);
}

void QrCodeTracker::display()
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(inputImage, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("Result", inputImage);
}

} //namespace ORB_SLAM
