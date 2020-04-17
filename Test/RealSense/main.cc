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
        // Pass the IR Left and Depth images to the SLAM system
        SLAM.TrackRGBD(realsense.getIRLeftMatrix(), realsense.getDepthMatrix(), realsense.getIRLeftTimestamp());
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
    // SLAM.SaveTrajectory("CameraTrajectory.dat");
    // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}