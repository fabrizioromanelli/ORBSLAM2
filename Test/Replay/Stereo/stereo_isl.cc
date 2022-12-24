/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
void ProgressBar(int, int);

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl << "Usage: ./stereo_isl path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file" << endl;
    cerr << endl << "Example: .Test/Stereo/stereo_isl /usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow /usr/local/share/ORB_SLAM2/Config/Zed-M-Stereo.yaml ~/workspace/datasets/isl_01/left ~/workspace/datasets/isl_01/right ~/workspace/datasets/isl_01/timestamps.txt" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimeStamp;
  LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft, vstrImageRight, vTimeStamp);

  if(vstrImageLeft.empty() || vstrImageRight.empty())
  {
    cerr << "ERROR: No images in provided path." << endl;
    return 1;
  }

  if(vstrImageLeft.size()!=vstrImageRight.size())
  {
    cerr << "ERROR: Different number of left and right images." << endl;
    return 1;
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, false, false, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imRight, imLeftRect, imRightRect;
  for(int ni=0; ni<nImages; ni++)
  {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

    if(imLeft.empty())
    {
      cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

    if(imRight.empty())
    {
      cerr << endl << "Failed to load image at: " << string(vstrImageRight[ni]) << endl;
      return 1;
    }

    double tframe = vTimeStamp[ni];

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni]=ttrack;

    // Wait to load the next frame
    double T=0;
    if(ni<nImages-1)
      T = vTimeStamp[ni+1]-tframe;
    else if(ni>0)
      T = tframe-vTimeStamp[ni-1];

    if(ttrack<T)
      std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));

    ProgressBar(ni, nImages);
  }

  cout << endl;

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for(int ni=0; ni<nImages; ni++)
  {
    totaltime+=vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  cout << "mean tracking time: " << totaltime/nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

  return 0;
}

void ProgressBar(int step, int totalSteps) {
  static float progress;
  const int resolution = 1000;

  if (step == 0) {
    progress = 0.0;
  }

  if (step % (totalSteps / resolution) == 0) {
    // Progress bar
    int barWidth = 70;
    cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
      if (i < pos) cout << "=";
      else if (i == pos) cout << ">";
      else cout << " ";
    }
    cout << "] " << fixed << setw(3) << setprecision(1) << progress * 100.0 << " %\r";
    cout.flush();
    progress += 1/float(resolution);
  }
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(12000);
  vstrImageLeft.reserve(12000);
  vstrImageRight.reserve(12000);
  while(!fTimes.eof())
  {
    string s;
    getline(fTimes,s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + "_left.jpg");
      vstrImageRight.push_back(strPathRight + "/" + ss.str() + "_right.jpg");
      double t;
      ss >> t;
      vTimeStamps.push_back(t/1e9);
    }
  }
}
