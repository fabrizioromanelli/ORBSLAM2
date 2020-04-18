#include <string>
#include <chrono>
#include <thread>

#include "realsense.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  try {
    RealSense::sModality mode = RealSense::IRD;
    RealSense realsense(mode);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    // Main loop
    for(;;)
    {
      realsense.run();

      // Showing images
      cv::Mat irFrame    = realsense.getIRLeftMatrix();
      cv::Mat depthFrame = realsense.getDepthMatrix();
      namedWindow("IR Left", WINDOW_AUTOSIZE);
      imshow("IR Left", irFrame);
      namedWindow("Depth", WINDOW_AUTOSIZE);
      imshow("Depth", depthFrame);

      // Saving files
      char filename_ir_[50] = "./infrared/ir_";
      char *filename_ir = &filename_ir_[0];
      strcat(filename_ir, to_string(realsense.getIRLeftTimestamp()).c_str());
      strcat(filename_ir, ".jpg");
      imwrite(filename_ir, irFrame);
      depthFrame.convertTo(depthFrame, CV_8UC1, 15 / 256.0);
      char filename_depth_[50] = "./depth/depth_";
      char *filename_depth = &filename_depth_[0];
      strcat(filename_depth, to_string(realsense.getIRLeftTimestamp()).c_str());
      strcat(filename_depth, ".jpg");
      imwrite(filename_depth, depthFrame);

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


  // for (;;)
  // {
  //   frames = pipe.wait_for_frames();

  //   // Get each frame
  //   rs2::frame depth_frame = frames.get_depth_frame();
  //   rs2_frame * frameP     = depth_frame.get();

  //   // Get frame timestamp
  //   rs2_time_t frame_timestamp = rs2_get_frame_timestamp(frameP, &e);
  //   // cout << std::fixed << std::setw(11) << std::setprecision(6) <<  "Timestamp: " << frame_timestamp << endl;

  //   // Creating OpenCV Matrix from a color image
  //   Mat depth_img(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

  //   // Convert 16bit image to 8bit image
  //   depth_img.convertTo(depth_img, CV_8UC1, 15 / 256.0);

  //   // Display in a GUI
  //   namedWindow("Grab Images", WINDOW_AUTOSIZE);
  //   imshow("Grab Images", depth_img);

    // char filename_[50] = "calib_";
    // char *filename = &filename_[0];

    // strcat(filename, to_string(frame_timestamp).c_str());
    // strcat(filename, ".jpg");

    // int key = waitKey(10);
    // // Save image to jpeg if Spacebar has been pressed
    // if( key == 32 ) {
    //   imwrite(filename, depth_img);
  //   } else if (key == 27) { // stop capturing by pressing ESC
  //     break;
  //   }
  // }

  return 0;
}
