#include <string>

// include OpenCV header file
#include <opencv2/opencv.hpp>

#include "realsense.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  try {
    cout << "Initializing Intel RealSense..." << endl;
    RealSense realsense(RealSense::IRD);
    cout << "Intel RealSense correctly initialized!" << endl;

    // Main loop
    for(;;)
    {
      realsense.run();
      Mat irMatrix    = realsense.getIRLeftMatrix();
      Mat depthMatrix = realsense.getDepthMatrix();

      // Display in a GUI
      namedWindow("IR Left Stream", WINDOW_AUTOSIZE);
      imshow("IR Left Stream", irMatrix);
      namedWindow("Depth Stream", WINDOW_AUTOSIZE);
      imshow("Depth Stream", depthMatrix);

      // char filename_[50] = "calib_";
      // char *filename = &filename_[0];

      // strcat(filename, to_string(realsense.getIRLeftTimestamp()).c_str());
      // strcat(filename, ".jpg");

      // int key = waitKey(10);
      // if( key == 32 ) {
      //   imwrite(filename, ir_left);
      // } else if (key == 27) {
      //   break;
      // }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}
