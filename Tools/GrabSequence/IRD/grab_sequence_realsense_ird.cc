#include <string>
#include <chrono>
#include <thread>
#include <sys/stat.h>

#include "realsense.h"

using namespace std;
using namespace cv;

bool save_frame_raw_data(const std::string& filename, rs2::frame frame);

int main(int argc, char** argv)
{
  try {
    RealSense::sModality mode = RealSense::IRD;
    RealSense realsense(mode);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Main loop
    for(;;)
    {
      realsense.run();

      // Showing images
      cv::Mat irMatrix    = realsense.getIRLeftMatrix();
      cv::Mat depthMatrix = realsense.getDepthMatrix();
      namedWindow("IR Left", WINDOW_AUTOSIZE);
      imshow("IR Left", irMatrix);
      namedWindow("Depth", WINDOW_AUTOSIZE);
      imshow("Depth", depthMatrix);

      // Saving files
      char filename_ir_[50] = "./infrared/ir_";
      char *filename_ir = &filename_ir_[0];
      strcat(filename_ir, to_string(realsense.getIRLeftTimestamp()).c_str());
      strcat(filename_ir, ".jpg");
      imwrite(filename_ir, irMatrix);

      // depthMatrix.convertTo(depthMatrix, CV_16UC1);

      char filename_depth_[50] = "./depth/depth_";
      char *filename_depth = &filename_depth_[0];
      strcat(filename_depth, to_string(realsense.getIRLeftTimestamp()).c_str());
      strcat(filename_depth, ".png");
      imwrite(filename_depth, depthMatrix);

      int key = waitKey(10);
      // Stop acquisition process when Spacebar is pressed
      if( key == 32 ) {
        break;
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}
