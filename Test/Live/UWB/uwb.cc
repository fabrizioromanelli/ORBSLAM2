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
#include <sys/stat.h>

#include "uwb.h"
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define VSLAM_FREQUENCY 15.0 // Hz
#define UWB_FREQUENCY    2.0 // Hz
// #define DEBUG
#define NO_UWB

void saveUWBreadings(const string &, vector<double>, vector<vector<uint16_t>>);

int main(int argc, char **argv)
{
  if(argc != 7)
  {
    cerr << endl << "Usage: ./uwb" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  try {
    // Initialize sensors
    RealSense realsense(RealSense::IRD, (uint32_t)VSLAM_FREQUENCY);
    #ifndef NO_UWB
      initialize_UWB();
    #endif

    // Clone parameters from command line
    bool display = false;
    string displayS = string(argv[3]);
    if(displayS.compare("ON") == 0)
      display = true;

    bool saveFile = false;
    string saveFileS = string(argv[4]);
    if(saveFileS.compare("ON") == 0)
      saveFile = true;

    bool autoclose = false;
    string autocloseS = string(argv[5]);
    if(autocloseS.compare("ON") == 0)
      autoclose = true;

    bool printTraj = false;
    string printTrajS = string(argv[6]);
    if(printTrajS.compare("ON") == 0)
      printTraj = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, display, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    double elapsedTime = 1.0/UWB_FREQUENCY; // time elapsed for synching UWB with VSLAM readings (s)
    chrono::steady_clock::time_point tSlam_start;
    chrono::steady_clock::time_point tSlam_end;
    chrono::steady_clock::time_point tSlam_prev  = chrono::steady_clock::now();
    chrono::steady_clock::time_point tUwb_prev;

    vector<double> uwbTimestamps;
    vector<vector<uint16_t>> uwbReadings;

    // Main loop
    for(;;)
    {
      tSlam_start = chrono::steady_clock::now();
      #ifdef DEBUG
        cout << "VSLAM frequency: " << 1/chrono::duration_cast<chrono::duration<double> >(tSlam_start - tSlam_prev).count() << "Hz" << endl;
      #endif
      tSlam_prev = chrono::steady_clock::now();

      realsense.run();
      // cout << fixed << setw(11) << setprecision(6) << "Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
      cv::Mat irMatrix    = realsense.getIRLeftMatrix();
      cv::Mat depthMatrix = realsense.getDepthMatrix();

      // Pass the IR Left and Depth images to the SLAM system
      cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());

      // Computing Visual odometry covariance matrix
      if (SLAM.GetTrackingState() == Tracking::OK)
      {
        vector<MapPoint*> trackedPoints = SLAM.GetTrackedMapPoints();
        std::vector<cv::Mat> featPos;
        for (size_t i = 0; i < trackedPoints.size(); i++)
        {
          MapPoint* pMP = trackedPoints[i];
          if(pMP)
          {
            // std::cout << "[map]: " << pMP->GetWorldPos() << std::endl;
            featPos.push_back(pMP->GetWorldPos());
          }
        }

        cv::Mat lambda = cv::Mat::zeros(2,2,CV_32F);
        cv::Mat factor = cv::Mat::zeros(6,6,CV_32F);
        float fx = 379.895904541016 / irMatrix.cols; // expressed in meters
        float fy = 379.895904541016 / irMatrix.rows; // expressed in meters
        lambda.at<float>(0,0) = 1.0/(fx*fx);
        lambda.at<float>(1,1) = 1.0/(fy*fy);
        cv::Mat Rcg = cv::Mat::zeros(3,3,CV_32F); // Rotation from Camera pose to Global frame
        Rcg.at<float>(0,0) = cameraPose.at<float>(0,0);
        Rcg.at<float>(0,1) = cameraPose.at<float>(0,1);
        Rcg.at<float>(0,2) = cameraPose.at<float>(0,2);
        Rcg.at<float>(1,0) = cameraPose.at<float>(1,0);
        Rcg.at<float>(1,1) = cameraPose.at<float>(1,1);
        Rcg.at<float>(1,2) = cameraPose.at<float>(1,2);
        Rcg.at<float>(2,0) = cameraPose.at<float>(2,0);
        Rcg.at<float>(2,1) = cameraPose.at<float>(2,1);
        Rcg.at<float>(2,2) = cameraPose.at<float>(2,2);

        for (size_t j = 0; j < featPos.size(); j++)
        {
          cv::Mat H1 = cv::Mat::zeros(2,3,CV_32F);
          cv::Mat H2 = cv::Mat::zeros(3,6,CV_32F);
          cv::Mat Pcf = cv::Mat::zeros(3,1,CV_32F); // Pose of feature in respect to Camera frame
          cv::Mat Pgc = cv::Mat::zeros(3,1,CV_32F);; // Pose of camera in respect to Global frame
          float a, b;
          Pgc.at<float>(0) = cameraPose.at<float>(0,3);
          Pgc.at<float>(1) = cameraPose.at<float>(1,3);
          Pgc.at<float>(2) = cameraPose.at<float>(2,3);

          Pcf = Rcg * (featPos[j] - Pgc);

          if (Pcf.at<float>(2) == 0.0f)
          {
            a = 1000000;
            b = 1000000000000;
          }
          else
          {
            a = 1/Pcf.at<float>(2);
            b = 1/(Pcf.at<float>(2)*Pcf.at<float>(2));
          }
          H1.at<float>(0,0) = a;
          H1.at<float>(0,2) = -Pcf.at<float>(0)*b;
          H1.at<float>(1,1) = a;
          H1.at<float>(1,2) = -Pcf.at<float>(1)*b;
          H2.at<float>(0,1) = -Pcf.at<float>(2);
          H2.at<float>(0,2) = Pcf.at<float>(1);
          H2.at<float>(0,3) = -Rcg.at<float>(0,0);
          H2.at<float>(0,4) = -Rcg.at<float>(0,1);
          H2.at<float>(0,5) = -Rcg.at<float>(0,2);
          H2.at<float>(1,0) = Pcf.at<float>(2);
          H2.at<float>(1,2) = -Pcf.at<float>(0);
          H2.at<float>(1,3) = -Rcg.at<float>(1,0);
          H2.at<float>(1,4) = -Rcg.at<float>(1,1);
          H2.at<float>(1,5) = -Rcg.at<float>(1,2);
          H2.at<float>(2,0) = -Pcf.at<float>(1);
          H2.at<float>(2,1) = Pcf.at<float>(0);
          H2.at<float>(2,3) = -Rcg.at<float>(2,0);
          H2.at<float>(2,4) = -Rcg.at<float>(2,1);
          H2.at<float>(2,5) = -Rcg.at<float>(2,2);

          cv::Mat tmp = H1*H2;
          factor = factor + tmp.t() * lambda.inv() * tmp;
        }
        std::cout << factor.inv() << std::endl;
      }

      tSlam_end = chrono::steady_clock::now();

      double ttrack = chrono::duration_cast<chrono::duration<double> >(tSlam_end - tSlam_start).count();

#ifndef NO_UWB
      // Call UWB readings after 500ms have passed from the previous scan
      if(elapsedTime >= 1.0/UWB_FREQUENCY)
      {
        if(read_UWB())
        {
          uint16_t *uwb_distances;
          uwb_distances = get_UWB_dist();
          // cout << fixed << setw(11) << setprecision(6) << "realsense.getIRLeftTimestamp(): " << realsense.getIRLeftTimestamp() << endl;
          uwbTimestamps.push_back(realsense.getIRLeftTimestamp());
          vector<uint16_t> tmp;
          for(int i = 0; i < 6; i++)
          {
            tmp.push_back(uwb_distances[i]);
          }
          uwbReadings.push_back(tmp);
          elapsedTime = 0.0;
        }
        else
        {
          cout << "Error while reading UWBs!" << endl << flush;
        }
      }
      else
      {
        elapsedTime += chrono::duration_cast<chrono::duration<double> >(tSlam_end - tUwb_prev).count();
        // cout << "elapsedTime: " << elapsedTime << "s" << endl;
      }

      tUwb_prev = chrono::steady_clock::now();
#endif

      #ifdef DEBUG
        cout << "Track frequency: " << 1/ttrack << "Hz" << endl;
      #endif

      if (printTraj)
        cout << "Camera position" << cameraPose << endl;

      // Saving files
      if (saveFile) {
        char filename_ir_[50] = "./infrared/ir_";
        char *filename_ir = &filename_ir_[0];
        strcat(filename_ir, to_string(realsense.getIRLeftTimestamp()).c_str());
        strcat(filename_ir, ".jpg");
        imwrite(filename_ir, irMatrix);

        // depthMatrix.convertTo(depthMatrix, CV_8UC1, 15 / 256.0);

        char filename_depth_[50] = "./depth/depth_";
        char *filename_depth = &filename_depth_[0];
        strcat(filename_depth, to_string(realsense.getIRLeftTimestamp()).c_str());
        strcat(filename_depth, ".png");
        imwrite(filename_depth, depthMatrix);
        }

      int key = waitKey(1);
      // Stop SLAM when Spacebar is pressed or if the map changed (so a loop has been closed)
      if( key == 32 || (SLAM.MapChanged() && autoclose)) {
        cout << "Loop closed ==> shutting down SLAM" << endl;
        break;
      }

      // Sleep according to the VSLAM frequency
      // if(1/ttrack > VSLAM_FREQUENCY)
      //   this_thread::sleep_for(chrono::microseconds(static_cast<size_t>((1/VSLAM_FREQUENCY-ttrack)*1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.dat");

    // Save UWB readings
    saveUWBreadings("UWBReadings.dat", uwbTimestamps, uwbReadings);
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

void saveUWBreadings(const string &filename, vector<double> timestamps, vector<vector<uint16_t>> readings)
{
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  vector<vector<uint16_t>>::iterator itReadings = readings.begin();

  for(vector<double>::iterator itTimestamps = timestamps.begin(); itTimestamps != timestamps.end(); itTimestamps++, itReadings++)
  {
    vector<uint16_t> _readings = *itReadings;
    f << setprecision(6) << *itTimestamps << " " <<  setprecision(9) << _readings.at(0) << " " << _readings.at(1) << " " << _readings.at(2) << " " << _readings.at(3) << " " << _readings.at(4) << " " << _readings.at(5) << endl;
  }
  f.close();
  cout << endl << "UWB readings saved!" << endl;
}