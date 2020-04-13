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

int main(int argc, char **argv)
{
  if(argc != 3)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings" << endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true, true);

  cout << endl << "-------" << endl;
  cout << "Start processing video stream ..." << endl;

  try {
    RealSense realsense;

    // Main loop
    for(;;)
    {
      realsense.updateAlign();
      // cout << fixed << setw(11) << setprecision(6) <<  "getRGBAlignedTimestamp  : " << realsense.getRGBAlignedTimestamp() << endl;
      // cout << fixed << setw(11) << setprecision(6) <<  "getDepthAlignedTimestamp: " << realsense.getDepthAlignedTimestamp() << endl;

      // Pass the image to the SLAM system
      SLAM.TrackRGBD(realsense.getColorMatrix(), realsense.getDepthMatrix(), realsense.getRGBAlignedTimestamp());

      int key = cv::waitKey(10);
      // Stop SLAM when Spacebar is pressed
      if( key == 32 ) {
        break;
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  // SLAM.SaveTrajectory("CameraTrajectory.dat");
  // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");

  return 0;
}