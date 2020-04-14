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
  if(argc != 3)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings" << endl;
    return 1;
  }

  try {
    RealSense realsense(RealSense::RGBD);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, true, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    // Main loop
    for(;;)
    {
      realsense.run();

      if (realsense.isValidAlignedFrame()) {
        // cout << fixed << setw(11) << setprecision(6) << "getRGBTimestamp   : " << realsense.getRGBTimestamp() << endl;
        // cout << fixed << setw(11) << setprecision(6) << "getDepthTimestamp : " << realsense.getDepthTimestamp() << endl;

        cout << fixed << setw(11) << setprecision(6) << "Frame Displacement: " << realsense.getTemporalFrameDisplacement() << endl;
        cout << fixed << setw(11) << setprecision(6) << "Average Timestamp : " << realsense.getAverageTimestamp() << endl;
      }

      // if (realsense.isValidAlignedFrame()) {
      //   // Pass the RGB and Depth images to the SLAM system
      //   SLAM.TrackRGBD(realsense.getColorMatrix(), realsense.getDepthMatrix(), realsense.getAverageTimestamp());
      // } else {
      //   cout << "Frames are not time consistent";
      // }

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