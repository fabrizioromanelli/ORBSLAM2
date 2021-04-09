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
  parameters = aruco::DetectorParameters::create();
  dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
  loadArucoCodeList();
}

// Scan for any Aruco code in the image
void ArucoCodeScanner::Scan(Mat _inputImage)
{
  inputImage = _inputImage;

  aruco::detectMarkers(_inputImage, dictionary, arucoBboxes, arucoIds, parameters, rejectedCandidates);

  try
  {
    for(size_t i = 0; i < arucoBboxes.size(); i++)
    {
      Rect r = boundingRect(arucoBboxes[i]);
      Point2f center(r.x+r.width/2, r.y+r.height/2);
      arucoCenters.push_back(center);
    }
  }
  catch(const std::exception& e){}
}

// Scan for any Aruco code in the image pruning arucoBboxes and arucoIds
// from the codes that are not in the list arucoCodes.dat.
bool ArucoCodeScanner::Detect(Mat _inputImage)
{
  vector<int>::iterator idIt;
  vector<vector<Point2f>>::iterator bboxesIt;

  this->Scan(_inputImage);

  for(idIt = arucoIds.begin(), bboxesIt = arucoBboxes.begin(); idIt != arucoIds.end();)
  {
    if (isArucoValid(*idIt))
    {
      ++idIt;
      ++bboxesIt;
    }
    else
    {
      arucoBboxes.erase(bboxesIt);
      arucoIds.erase(idIt);
    }
  }

  if (arucoIds.size() != 0) return true;
  else return false;
}

void ArucoCodeScanner::loadArucoCodeList()
{
  std::ifstream in("/usr/local/share/ORB_SLAM2/arucoCodes.dat");

  std::string line;
  while (std::getline(in, line))
  {
    std::istringstream iss(line);
    ArucoCode validArucoCode(stoi(line));
    arucoValidCodes.push_back(validArucoCode);
  }
}

bool ArucoCodeScanner::isArucoValid(int currentCode)
{
  for(vector<ArucoCode>::iterator it = arucoValidCodes.begin(); it!=arucoValidCodes.end(); it++)
    if (it->getCode() == currentCode)
      return true;

  return false;
}

vector<vector<Point2f>> ArucoCodeScanner::getBoundingBoxes()
{
  return(arucoBboxes);
}

vector<Point2f> ArucoCodeScanner::getBoundingBoxCenters()
{
  return(arucoCenters);
}

void ArucoCodeScanner::display()
{
  Mat outputImage = inputImage.clone();
  aruco::drawDetectedMarkers(outputImage, arucoBboxes, arucoIds);
  imshow("Result", outputImage);
}

void ArucoCodeScanner::getImage(cv::Mat& img)
{
  Mat outputImage = inputImage.clone();
  aruco::drawDetectedMarkers(outputImage, arucoBboxes, arucoIds);
  img = outputImage;
}

} //namespace ORB_SLAM
