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

    bool firstReset = true;

    for(;;)
    {
      realsense.run();
      rs2_pose pose = realsense.getPose();

      cv::Mat irMatrix    = realsense.getIRLeftMatrix();
      cv::Mat depthMatrix = realsense.getDepthMatrix();
      // Pass the IR Left and Depth images to the SLAM system
      cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());
      // cv::Mat covMat = SLAM.GetCurrentCovarianceMatrix(fx, fy, cameraPose, true);

      if (printTraj && !cameraPose.empty())
      {
        // The first time I receive a valid ORB-SLAM2 sample, I have to reset
        // the T265 tracker.
        if (firstReset) {
          realsense.resetPoseTrack();
          realsense.run();
          pose = realsense.getPose();
          firstReset = false;
        }

        // cout.precision(2);
        // cout << fixed << setw(8) << "X: " << -cameraPose.at<float>(2,3) << setw(8) << " Y: " << -cameraPose.at<float>(0,3) << setw(8) << " Z: " << -cameraPose.at<float>(1,3) << endl;
        cv::Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat Twc = -Rwc * cameraPose.rowRange(0, 3).col(3);

        // Pose from Intel RealSense T265
        // Transforming translations from T265 to ORB-SLAM2 reference
        cv::Mat t265Position = cv::Mat(1, 3, CV_32F);
        t265Position.at<float>(0) = -pose.translation.z;
        t265Position.at<float>(1) = -pose.translation.x;
        t265Position.at<float>(2) = pose.translation.y;
        Eigen::Quaternionf q_t265(pose.rotation.w, -pose.rotation.z, pose.rotation.x, -pose.rotation.y);
        q_t265 = q_t265.conjugate();

        cout << fixed << setw(11) << setprecision(6) << "[ T265] " << q_t265.x() << ", " << q_t265.y() << ", " << q_t265.z() << ", " << q_t265.w() << endl;
        // cout << fixed << setw(11) << setprecision(6) << "[ T265] " << pose.rotation << endl;
        // cout << fixed << setw(11) << setprecision(6) << "[T265] " << realsense.getPoseTimestamp() << " " << t265Position << " " << pose.rotation << endl;
        // cout << fixed << setw(11) << setprecision(6) << "[T265] " << realsense.getPoseTimestamp() << " " << pose.translation << " " << pose.rotation << " " << pose.tracker_confidence << endl;

        // Pose from ORB-SLAM2 w/ Intel RealSense D435i
        Eigen::Matrix3f orMat;
        orMat(0,0) = Rwc.at<float>(0,0);
        orMat(0,1) = Rwc.at<float>(0,1);
        orMat(0,2) = Rwc.at<float>(0,2);
        orMat(1,0) = Rwc.at<float>(1,0);
        orMat(1,1) = Rwc.at<float>(1,1);
        orMat(1,2) = Rwc.at<float>(1,2);
        orMat(2,0) = Rwc.at<float>(2,0);
        orMat(2,1) = Rwc.at<float>(2,1);
        orMat(2,2) = Rwc.at<float>(2,2);
        Eigen::Quaternionf q_orb_tmp(orMat);
        Eigen::Quaternionf q_orb(q_orb_tmp.w(), -q_orb_tmp.z(), -q_orb_tmp.x(), -q_orb_tmp.y());

        cout << fixed << setw(11) << setprecision(6) << "[D435i] " << q_orb.x() << ", " << q_orb.y() << ", " << q_orb.z() << ", " << q_orb.w() << endl;
        // cout << fixed << setw(11) << setprecision(6) << "[D435] " <<  realsense.getIRLeftTimestamp() << " " << Twc.at<float>(2) << " " << -Twc.at<float>(0) << " " << -Twc.at<float>(1) << " " << q_orb.x() << " " << q_orb.y() << " " << q_orb.z() << " " << q_orb.w() << endl;
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