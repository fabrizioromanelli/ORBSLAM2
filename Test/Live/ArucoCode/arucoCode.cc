/**
* Test video stream with ORB-SLAM2 - Aruco Code tracker.
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
#include <ArucoCodeScanner.h>

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 1)
  {
    cerr << endl << "Usage: ./arucoCode" << endl;
    return 1;
  }

  try {
    RealSense realsense(RealSense::RGBD);
    ArucoCodeScanner *arucoCodeScanner = new ArucoCodeScanner();

    // Main loop
    for(;;)
    {
      realsense.run();

      arucoCodeScanner->Detect(realsense.getColorMatrix());
      arucoCodeScanner->display();

      int key = waitKey(10);
      // Stop everything
      if( key == 32 ) {
        cout << "Shutting down..." << endl;
        break;
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}