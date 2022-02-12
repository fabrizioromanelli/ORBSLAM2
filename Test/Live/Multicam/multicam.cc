/**
* Test video stream with D435i + T265 (and ORB-SLAM2).
*
*/

#include <signal.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <sys/stat.h>

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

System *pSLAM;

// Handling CTRL+C event
void my_handler(int s){
  pSLAM->Shutdown();
  sleep(5);
  exit(1);
}

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl << "Usage: ./multicam_live" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  try {
    RealSense::sModality mode = RealSense::MULTI;
    RealSense realsense(mode);

    bool saveFile = false;
    string saveFileS = string(argv[3]);
    if(saveFileS.compare("ON") == 0)
      saveFile = true;

    bool autoclose = false;
    string autocloseS = string(argv[4]);
    if(autocloseS.compare("ON") == 0)
      autoclose = true;

    bool printTraj = false;
    string printTrajS = string(argv[5]);
    if(printTrajS.compare("ON") == 0)
      printTraj = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, false, true);
    pSLAM = &SLAM;

    float fx = 379.895904541016 / 640; // expressed in meters
    float fy = 379.895904541016 / 480; // expressed in meters

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    for(;;)
    {
      realsense.run();
      rs2_pose pose = realsense.getPose();
      cout << fixed << setw(11) << setprecision(6) << realsense.getPoseTimestamp() << " " << pose.translation << " " << pose.rotation << " " << pose.tracker_confidence << endl;

      cout << fixed << setw(11) << setprecision(6) << "Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
      cv::Mat irMatrix    = realsense.getIRLeftMatrix();
      cv::Mat depthMatrix = realsense.getDepthMatrix();
      // Pass the IR Left and Depth images to the SLAM system
      cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());
      // cv::Mat covMat = SLAM.GetCurrentCovarianceMatrix(fx, fy, cameraPose, true);

      if (printTraj && !cameraPose.empty())
      {
        cout.precision(2);
        // cout << fixed << setw(8) << "X: " << -cameraPose.at<float>(2,3) << setw(8) << " Y: " << -cameraPose.at<float>(0,3) << setw(8) << " Z: " << -cameraPose.at<float>(1,3) << endl;
        cv::Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat Twc = -Rwc * cameraPose.rowRange(0, 3).col(3);
        cout << fixed << setprecision(5) << setw(8) << "X: " << Twc.at<float>(2) << setw(8) << " Y: " << -Twc.at<float>(0) << setw(8) << " Z: " << -Twc.at<float>(1) << endl;
      }

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


      int key = waitKey(10);
      // Stop SLAM when Spacebar is pressed or if the map changed (so a loop has been closed)
      if( key == 32 || (SLAM.MapChanged() && autoclose)) {
        cout << "Loop closed ==> shutting down SLAM" << endl;
        break;
      }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.dat");
  } catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}