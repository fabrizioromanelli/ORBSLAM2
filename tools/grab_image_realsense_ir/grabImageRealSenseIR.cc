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
    RealSense realsense(RealSense::IRL);
    cout << "Intel RealSense correctly initialized!" << endl;

    // realsense.enableLaser(40.0);

    // Main loop
    for(;;)
    {
      realsense.run();
      Mat ir_left = realsense.getIRLeftMatrix();

      // Display in a GUI
      namedWindow("Grab Images", WINDOW_AUTOSIZE);
      imshow("Grab Images", ir_left);

      char filename_[50] = "calib_";
      char *filename = &filename_[0];

      strcat(filename, to_string(realsense.getIRLeftTimestamp()).c_str());
      strcat(filename, ".jpg");

      int key = waitKey(10);
      if( key == 32 ) {
        imwrite(filename, ir_left);
      } else if (key == 27) {
        break;
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}
