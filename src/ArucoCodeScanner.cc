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

  cv::aruco::detectMarkers(_inputImage, dictionary, arucoBboxes, arucoIds, parameters, rejectedCandidates);

  try
  {
    for(size_t i = 0; i < arucoBboxes.size(); i++)
    {
      Rect r = boundingRect(arucoBboxes[i]);
      Point2f center(r.x+r.width/2, r.y+r.height/2);
      arucoCenters[i] = center;
    }
  }
  catch(const std::exception& e){}
}

// Scan for any Aruco code in the image and returns true if the code is
// in the list of "good" ones.
Point2f ArucoCodeScanner::Detect(cv::Mat _inputImage)
{
  Point2f temp;
  std::string decodedData;

  this->Scan(_inputImage);



  return(temp);
}

// istream& operator >> (istream &fi, cv::Point &p)
// {
//    char char1, char2, char3;
//    fi >> char1 >> p.x >>char2 >> p.y >> char3;
//    if (char1=='[' &&  char2==',' && char3==']')
//     return fi;
// }

// istream& operator >> (istream &fi, cv::Point2d &p)
// {
//    char char1, char2, char3;
//    fi >> char1 >> p.x >>char2 >> p.y >> char3;
//    if (char1=='[' &&  char2==',' && char3==']')
//     return fi;
// }

void ArucoCodeScanner::loadArucoCodeList()
{
  std::string empty, code;
  cv::Point2f bBoxCenter;
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

vector<Mat> ArucoCodeScanner::getBoundingBoxes()
{
  return(arucoBboxes);
}

vector<Point2f> ArucoCodeScanner::getBoundingBoxCenters()
{
  return(arucoCenters);
}

void ArucoCodeScanner::display()
{
  cv::Mat outputImage = inputImage.clone();
  cv::aruco::drawDetectedMarkers(outputImage, arucoBboxes, arucoIds);
  imshow("Result", outputImage);
}

} //namespace ORB_SLAM
