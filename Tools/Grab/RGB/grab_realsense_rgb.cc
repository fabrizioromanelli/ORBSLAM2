#include <string>

// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  rs2_error* e = 0;

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  rs2::frameset frames;
  for (int i = 0; i < 30; i++)
  {
    // Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();
  }

  for (;;)
  {
    frames = pipe.wait_for_frames();

    // Get each frame
    rs2::frame color_frame = frames.get_color_frame();
    rs2_frame * frameP     = color_frame.get();

    // Get frame timestamp
    rs2_time_t frame_timestamp = rs2_get_frame_timestamp(frameP, &e);
    // cout << std::fixed << std::setw(11) << std::setprecision(6) <<  "Timestamp: " << frame_timestamp << endl;

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    // Display in a GUI
    namedWindow("Grab Images", WINDOW_AUTOSIZE);
    imshow("Grab Images", color);

    char filename_[50] = "calib_";
    char *filename = &filename_[0];

    strcat(filename, to_string(frame_timestamp).c_str());
    strcat(filename, ".jpg");

    int key = waitKey(10);
    // Save image to jpeg if Spacebar has been pressed
    if( key == 32 ) {
      imwrite(filename, color);
    } else if (key == 27) { // stop capturing by pressing ESC
      break;
    }
  }

  return 0;
}