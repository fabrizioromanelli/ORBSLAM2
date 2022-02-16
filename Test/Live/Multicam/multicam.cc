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

bool running = true;

// Handling CTRL+C event
void my_handler(int s);
template <typename T>
void saveTrajectory(const string &filename, vector<T> &trajectory, vector<rs2_time_t> &timestamps);

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl << "Usage: ./multicam_live" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         save trajectories[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  try {
    System *pSLAM;
    RealSense::sModality mode = RealSense::MULTI;
    RealSense *realsense = new RealSense(mode);

    bool saveFile = false;
    string saveFileS = string(argv[3]);
    if(saveFileS.compare("ON") == 0)
      saveFile = true;

    bool saveTraj = false;
    string saveTrajS = string(argv[4]);
    if(saveTrajS.compare("ON") == 0)
      saveTraj = true;

    bool printTraj = false;
    string printTrajS = string(argv[5]);
    if(printTrajS.compare("ON") == 0)
      printTraj = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, false, true);
    pSLAM = &SLAM;

    // float fx = 379.895904541016 / 640; // expressed in meters
    // float fy = 379.895904541016 / 480; // expressed in meters

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    bool firstReset = true;

    vector<rs2_pose> t265Poses, orbPoses;
    vector<rs2_time_t> t265Ts, orbTs;

    for(;;)
    {
      realsense->run();
      rs2_pose pose = realsense->getPose();

      cv::Mat irMatrix    = realsense->getIRLeftMatrix();
      cv::Mat depthMatrix = realsense->getDepthMatrix();

      // Pass the IR Left and Depth frames to the SLAM system
      HPose cameraPose = SLAM.TrackIRD(irMatrix, depthMatrix, realsense->getIRLeftTimestamp());
      unsigned int ORBState = (SLAM.GetTrackingState() == Tracking::OK) ? 3 : 0;
      rs2_pose orbPose;
      orbPose.translation.x = cameraPose.GetTranslation()[0];
      orbPose.translation.y = cameraPose.GetTranslation()[1];
      orbPose.translation.z = cameraPose.GetTranslation()[2];
      orbPose.rotation.x    = cameraPose.GetRotation()[0];
      orbPose.rotation.y    = cameraPose.GetRotation()[1];
      orbPose.rotation.z    = cameraPose.GetRotation()[2];
      orbPose.rotation.w    = cameraPose.GetRotation()[3];
      orbPose.tracker_confidence = ORBState;

      if (!cameraPose.empty())
      {
        // The first time I receive a valid ORB-SLAM2 sample, I have to reset
        // the T265 tracker.
        if (firstReset) {
          realsense->resetPoseTrack();
          realsense->run();
          pose = realsense->getPose();
          firstReset = false;
        }

        if (saveTraj) {
          t265Poses.push_back(pose);
          orbPoses.push_back(orbPose);
          t265Ts.push_back(realsense->getPoseTimestamp());
          orbTs.push_back(realsense->getIRLeftTimestamp());
        }

        if (printTraj) {
          cout << fixed << setw(11) << setprecision(6) << "[D435i] " << realsense->getIRLeftTimestamp() << " " << orbPose.translation << " " << orbPose.rotation << " " << orbPose.tracker_confidence << endl;
          cout << fixed << setw(11) << setprecision(6) << "[ T265] " << realsense->getPoseTimestamp() << " " << pose.translation << " " << pose.rotation << " " << pose.tracker_confidence << endl;
        }
      }

      // Saving files
      if (saveFile) {
        char filename_ir_[50] = "./infrared/ir_";
        char *filename_ir = &filename_ir_[0];
        strcat(filename_ir, to_string(realsense->getIRLeftTimestamp()).c_str());
        strcat(filename_ir, ".jpg");
        imwrite(filename_ir, irMatrix);

        // depthMatrix.convertTo(depthMatrix, CV_8UC1, 15 / 256.0);

        char filename_depth_[50] = "./depth/depth_";
        char *filename_depth = &filename_depth_[0];
        strcat(filename_depth, to_string(realsense->getIRLeftTimestamp()).c_str());
        strcat(filename_depth, ".png");
        imwrite(filename_depth, depthMatrix);
      }

      if (!running) {
        break;
      }
    }

    pSLAM->Shutdown();
    sleep(5);
    delete(realsense);

    if (saveTraj) {
      saveTrajectory("t265.dat", t265Poses, t265Ts);
      saveTrajectory("orb.dat", orbPoses, orbTs);
    }
    // // Save camera trajectory
    // SLAM.SaveTrajectory("CameraTrajectory.dat");
  } catch(exception& ex) {
    cout << "Error! " << ex.what() << endl;
  }

  return 0;
}

// Handling CTRL+C event
void my_handler(int s){
  running = false;
}

template <typename T>
void saveTrajectory(const string &filename, vector<T> &trajectory, vector<rs2_time_t> &timestamps)
{
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

  ofstream f;
  f.open(filename.c_str());
  f << fixed << setw(11) << setprecision(6);

  vector<rs2_time_t>::iterator tsIt = timestamps.begin();
  if constexpr(is_same<vector<T>,vector<rs2_pose>>::value) {
    for (vector<rs2_pose>::iterator it = trajectory.begin(); it != trajectory.end(); ++it, ++tsIt) {
      f << *tsIt << ", " << (*it).translation << ", " << (*it).rotation << ", " << (*it).tracker_confidence << endl;
    }
  } else if constexpr(is_same<vector<T>,vector<HPose>>::value) {
    for (vector<HPose>::iterator it = trajectory.begin(); it != trajectory.end(); ++it, ++tsIt) {
      f << *tsIt << ", " << (*it).GetTranslation()[0] << ", " << (*it).GetTranslation()[1] << ", " << (*it).GetTranslation()[2] << ", " << (*it).GetRotation()[0] << ", " << (*it).GetRotation()[1] << ", " << (*it).GetRotation()[2] << ", " << (*it).GetRotation()[3] << endl;
    }
  } else {
    cout << "Unknown type" << endl;
  }

  f.close();
  cout << endl << "Trajectory saved!" << endl;
}