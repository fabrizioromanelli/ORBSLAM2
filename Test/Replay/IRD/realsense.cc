#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

void LoadImages(const string sequenceDir, vector<string> &imageFilenamesIR, vector<string> &imageFilenamesD, vector<double> &timestamps, const string depthExtension);

void ProgressBar(float progress);

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl << "Usage: ./realsense_replay" << endl 
                 << "         path_to_vocabulary_file" << endl
                 << "         path_to_configuration_file" << endl
                 << "         path_to_sequence" << endl
                 << "         depth_image_extension[jpg/png]" << endl
                 << "         display[ON/OFF]" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> imageFilenamesIR;
  vector<string> imageFilenamesD;
  vector<double> timestamps;
  string sequenceDir = string(argv[3]);
  string depthExtension = "."+string(argv[4]);
  LoadImages(sequenceDir, imageFilenamesIR, imageFilenamesD, timestamps, depthExtension);

  // Check consistency in the number of images and depthmaps
  int nImages = imageFilenamesIR.size();
  if(imageFilenamesIR.empty())
  {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  }
  else if(imageFilenamesD.size() != imageFilenamesIR.size())
  {
    cerr << endl << "Different number of images for ir and depth." << endl;
    return 1;
  }

  bool display = false;
  string displayS = string(argv[5]);
  if(displayS.compare("ON") == 0)
    display = true;

  try {
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, display, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat imIR, imD;
    // Main loop
    for(int ni = 0; ni < nImages; ni++)
    {
      // Read image and depthmap from file
      imIR = cv::imread(string(argv[3])+"/infrared/"+imageFilenamesIR[ni], cv::IMREAD_UNCHANGED);
      imD  = cv::imread(string(argv[3])+"/depth/"+imageFilenamesD[ni], cv::IMREAD_UNCHANGED);

      // The following lines work for images
      // half size the infrared ones. (e.g. 320x240)
      cv::Mat imDresized;
      cv::resize(imD, imDresized, cv::Size(), 2, 2, cv::INTER_CUBIC);

      // The following converts jpgs 8bit to 16bits matrices
      if (depthExtension.find(".jpg") == 0)
        imD.convertTo(imD, CV_16SC1, 256.0 / 15.0);

      double tframe = timestamps[ni]*132000;
      // SLAM.TrackRGBD(imIR, imD, tframe);
      // The following line works for images
      // half size the infrared ones. (e.g. 320x240)
      cv::Mat test = SLAM.TrackRGBD(imIR, imDresized, tframe);

      cout << "Camera pose: " << test << endl;

      ProgressBar((float)ni/nImages);
    }
    std::cout << std::endl;

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.dat");
    // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

void ProgressBar(float progress)
{
  int barWidth = 70;

  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; ++i) {
      if (i < pos) std::cout << "=";
      else if (i == pos) std::cout << ">";
      else std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0) << " %\r";
  std::cout.flush();
}

void ReadDirectory(const string& name, vector<string> &v)
{
  DIR* dirp = opendir(name.c_str());
  struct dirent * dp;
  while ((dp = readdir(dirp)) != NULL) {
    v.push_back(dp->d_name);
  }
  closedir(dirp);
}

void LoadImages(const string sequenceDir, vector<string> &imageFilenamesIR, vector<string> &imageFilenamesD, vector<double> &timestamps, const string depthExtension)
{
  ReadDirectory(sequenceDir + "/infrared", imageFilenamesIR);
  imageFilenamesIR.erase(imageFilenamesIR.begin());
  imageFilenamesIR.erase(imageFilenamesIR.begin());
  sort(imageFilenamesIR.begin(), imageFilenamesIR.end());

  ReadDirectory(sequenceDir + "/depth", imageFilenamesD);
  imageFilenamesD.erase(imageFilenamesD.begin());
  imageFilenamesD.erase(imageFilenamesD.begin());
  sort(imageFilenamesD.begin(), imageFilenamesD.end());

  for (auto x : imageFilenamesD)
  {
    // size_t sPos = x.find("depth_");
    // x.erase(sPos, 6);
    size_t sPos;
    sPos = x.find(depthExtension.c_str());
    x.erase(sPos, depthExtension.length()+1);
    timestamps.push_back(stod(x));
  }
}

// X: -0.003 Y: 0.013 Z: -0.028
// Camera pose: 
//  [0.99937624, 0.026254445, 0.023618288, -0.0032980845;
//  -0.025839692, 0.99950945, -0.017697753, 0.013493983;
//  -0.024071345, 0.017076425, 0.99956441, -0.027921962;
//  0, 0, 0, 1]