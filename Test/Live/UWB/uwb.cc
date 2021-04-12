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
    // initialize_UWB();

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
    vector<uint16_t[6]> uwbReadings;

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

      tSlam_end = chrono::steady_clock::now();

      double ttrack = chrono::duration_cast<chrono::duration<double> >(tSlam_end - tSlam_start).count();

      // Here we have to call for UWB readings only at least every 500ms
      if(elapsedTime >= 1.0/UWB_FREQUENCY)
      {
        if(/*read_UWB()*/true)
        {
          // uint16_t *uwd_distances;
          // uwd_distances = get_UWB_dist();
          // cout << fixed << setw(11) << setprecision(6) << "realsense.getIRLeftTimestamp(): " << realsense.getIRLeftTimestamp() << endl;
          uwbTimestamps.push_back(realsense.getIRLeftTimestamp());
          uint16_t uwd_distances[6];
          for(int i = 0; i < 6; i++)
          {
            uwd_distances[i] = rand();
          }
          uwbReadings.push_back(uwd_distances);
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
        cout << "elapsedTime: " << elapsedTime << "s" << endl;
      }

      tUwb_prev = chrono::steady_clock::now();

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
    // TODO
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}