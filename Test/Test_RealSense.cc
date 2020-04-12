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

// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  if(argc != 3)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings" << endl;
    return 1;
  }

  // Start the RealSense up
  rs2_error* e = 0;

  //Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  //Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  //Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  //Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  rs2::frameset frames;
  for (int i = 0; i < 30; i++)
  {
    // Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, true);

  cout << endl << "-------" << endl;
  cout << "Start processing video stream ..." << endl;

  // Main loop
  for(;;)
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

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(color, frame_timestamp);

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

  return 0;
}