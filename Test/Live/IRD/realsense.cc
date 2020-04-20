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

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings mode" << endl;
    return 1;
  }

  try {
    RealSense::sModality mode = RealSense::RGBD;
    if (strcmp(argv[3], "RGBD") == 0)
      mode = RealSense::RGBD;
    else if (strcmp(argv[3], "IRD") == 0)
      mode = RealSense::IRD;

    RealSense realsense(mode);
    // realsense.enableLaser(40.0);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, true, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
    {
      cerr << "Error creating directory infrared!" << endl;
      exit(1);
    }

    dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
    {
      cerr << "Error creating directory depth!" << endl;
      exit(1);
    }

    // Main loop
    for(;;)
    {
      realsense.run();

      if (mode == RealSense::RGBD) {
        if (realsense.isValidAlignedFrame()) {
          // cout << fixed << setw(11) << setprecision(6) << "RGB Timestamp     : " << realsense.getRGBTimestamp() << endl;
          // cout << fixed << setw(11) << setprecision(6) << "Depth Timestamp   : " << realsense.getDepthTimestamp() << endl;
          // cout << fixed << setw(11) << setprecision(6) << "Frame Displacement: " << realsense.getTemporalFrameDisplacement() << endl;
          // cout << fixed << setw(11) << setprecision(6) << "Average Timestamp : " << realsense.getAverageTimestamp() << endl;
          // Pass the RGB and Depth images to the SLAM system
          SLAM.TrackRGBD(realsense.getColorMatrix(), realsense.getDepthMatrix(), realsense.getAverageTimestamp());
        } else {
          cerr << "Frames are not time consistent" << endl;
        }
      } else if (mode == RealSense::IRD) {
        // cout << fixed << setw(11) << setprecision(6) << "Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
        cv::Mat irMatrix    = realsense.getIRLeftMatrix();
        cv::Mat depthMatrix = realsense.getDepthMatrix();
        // Pass the IR Left and Depth images to the SLAM system
        SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());

        // Saving files
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
      // Stop SLAM when Spacebar is pressed
      if( key == 32 ) {
        break;
      }
    }

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