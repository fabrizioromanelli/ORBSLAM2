/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <dirent.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

void printCPUinfo();
void LoadImages(const string sequenceDir, vector<string> &imageFilenamesIR, vector<string> &imageFilenamesD, vector<double> &timestamps, const string depthExtension);

int main(int argc, char **argv)
{
  if(argc != 5)
  {
    cerr << endl << "Usage: ./benchmark_ird_replay path_to_vocabulary_file path_to_configuration_file path_to_sequence depth_image_extension" << endl;
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

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  try {
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, false, true);

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
      // The following converts jpgs 8bit to 16bits matrices
      if (depthExtension.find(".jpg") == 0)
        imD.convertTo(imD, CV_16SC1, 256.0 / 15.0);

      double tframe = timestamps[ni];

#ifdef COMPILEDWITHC11
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
#else
      chrono::monotonic_clock::time_point t1 = chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      SLAM.TrackRGBD(imIR, imD, tframe);

#ifdef COMPILEDWITHC11
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
#else
      chrono::monotonic_clock::time_point t2 = chrono::monotonic_clock::now();
#endif

      // integral duration: requires duration_cast
      auto int_ms = chrono::duration_cast<chrono::milliseconds>(t2 - t1);

      cout << "Image #" << ni << " SLAM.TrackIRD duration: " << int_ms.count() << "ms" << endl;
    }

    // Stop all threads
    SLAM.Shutdown();
    // Get CPU information and prints them out
    printCPUinfo();
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

void printCPUinfo()
{
    FILE *cpuinfo = fopen("/proc/cpuinfo", "rb");
    char *arg = 0;
    size_t size = 0;
    while(getdelim(&arg, &size, 0, cpuinfo) != -1)
    {
       puts(arg);
    }
    free(arg);
    fclose(cpuinfo);
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
    size_t sPos = x.find("depth_");
    x.erase(sPos, 6);
    sPos = x.find(depthExtension.c_str());
    x.erase(sPos, 4);
    timestamps.push_back(stod(x));
  }
}