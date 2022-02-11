/**
* Test video stream with D435i + T265 (and ORB-SLAM2).
*
*/

#include <signal.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <sys/stat.h>

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

System *pSLAM;

// Handling CTRL+C event
void my_handler(int s){
  pSLAM->Shutdown();
  sleep(5);
  exit(1);
}

int main(int argc, char **argv)
{
  if(argc != 8)
  {
    cerr << endl << "Usage: ./multicam_live" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         mode[RGBD/IRD]" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  RealSense::sModality mode = RealSense::MULTI;
  RealSense realsense(mode);

  // for(;;)
  // {
  //   realsense.run();
  //   // rs2_pose pose = realsense.getPose();
  //   // cout << fixed << setw(11) << setprecision(6) << pose.translation << " " << pose.rotation << " " << pose.tracker_confidence << endl;
  // }

  // rs2::context ctx; // Create librealsense context for managing devices
  // vector<rs2::pipeline> pipelines;

  // // Capture serial numbers before opening streaming
  // vector<string> serials, names;
  // for (auto&& dev : ctx.query_devices())
  // {
  //   serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  //   names.push_back(dev.get_info(RS2_CAMERA_INFO_NAME));
  // }

  // size_t i = 0;
  // // Start a streaming pipe per each connected device
  // for (auto&& serial : serials)
  // {
  //   rs2::pipeline pipe(ctx);
  //   rs2::config cfg;
  //   cfg.enable_device(serial);
  //   pipe.start(cfg);
  //   pipelines.emplace_back(pipe);
  //   cout << "Connecting to device: " << names[i++] << " serial: " << serial << endl;
  // }

  // vector<double> timestamps;

  // int j = 0;

  // // Main app loop
  // while (j < 100)
  // {
  //   // Collect the new frames from all the connected devices
  //   vector<rs2::frame> new_frames;
  //   for (auto &&pipe : pipelines)
  //   {
  //     rs2_pose pose;
  //     auto frames = pipe.wait_for_frames();
  //     auto pose_frame = frames.get_pose_frame();
  //     if (pose_frame) {
  //       pose = pose_frame.get_pose_data();
  //       cout << fixed << setw(11) << setprecision(6) << pose.translation << " " << pose.rotation << " " << pose.tracker_confidence << " " << pose_frame.get_timestamp() << endl;
  //       j++;
  //     }
  //     auto color_frame = frames.get_color_frame();
  //     if (color_frame) {
  //       int ttt = color_frame.get_height();
  //       cout << fixed << setw(11) << setprecision(6) << ttt << " " << color_frame.get_timestamp() << endl;
  //       j++;
  //     }
  //   }
  // }

  // try {
  //   RealSense::sModality mode = RealSense::RGBD;
  //   if (strcmp(argv[3], "RGBD") == 0)
  //     mode = RealSense::RGBD;
  //   else if (strcmp(argv[3], "IRD") == 0)
  //     mode = RealSense::IRD;

  //   RealSense realsense(mode);
  //   // realsense.enableLaser(40.0);

  //   // Clone parameters from command line
  //   bool display = false;
  //   string displayS = string(argv[4]);
  //   if(displayS.compare("ON") == 0)
  //     display = true;

  //   bool saveFile = false;
  //   string saveFileS = string(argv[5]);
  //   if(saveFileS.compare("ON") == 0)
  //     saveFile = true;

  //   bool autoclose = false;
  //   string autocloseS = string(argv[6]);
  //   if(autocloseS.compare("ON") == 0)
  //     autoclose = true;

  //   bool printTraj = false;
  //   string printTrajS = string(argv[7]);
  //   if(printTrajS.compare("ON") == 0)
  //     printTraj = true;

  //   // Create SLAM system. It initializes all system threads and gets ready to process frames.
  //   System SLAM(argv[1], argv[2], System::RGBD, display, true);
  //   pSLAM = &SLAM;

  //   float fx = 379.895904541016 / 640; // expressed in meters
  //   float fy = 379.895904541016 / 480; // expressed in meters

  //   cout << endl << "-------" << endl;
  //   cout << "Start processing video stream ..." << endl;

  //   if (saveFile)
  //   {
  //     int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  //     dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  //   }

  //   // Main loop
  //   for(;;)
  //   {
  //     realsense.run();

  //     if (mode == RealSense::RGBD) {
  //       if (realsense.isValidAlignedFrame()) {
  //         // cout << fixed << setw(11) << setprecision(6) << "RGB Timestamp     : " << realsense.getRGBTimestamp() << endl;
  //         // cout << fixed << setw(11) << setprecision(6) << "Depth Timestamp   : " << realsense.getDepthTimestamp() << endl;
  //         // cout << fixed << setw(11) << setprecision(6) << "Frame Displacement: " << realsense.getTemporalFrameDisplacement() << endl;
  //         // cout << fixed << setw(11) << setprecision(6) << "Average Timestamp : " << realsense.getAverageTimestamp() << endl;
  //         // Pass the RGB and Depth images to the SLAM system
  //         SLAM.TrackRGBD(realsense.getColorMatrix(), realsense.getDepthMatrix(), realsense.getAverageTimestamp());
  //       } else {
  //         cerr << "Frames are not time consistent" << endl;
  //       }
  //     } else if (mode == RealSense::IRD) {
  //       // cout << fixed << setw(11) << setprecision(6) << "Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
  //       cv::Mat irMatrix    = realsense.getIRLeftMatrix();
  //       cv::Mat depthMatrix = realsense.getDepthMatrix();
  //       // Pass the IR Left and Depth images to the SLAM system
  //       cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());
  //       cv::Mat covMat = SLAM.GetCurrentCovarianceMatrix(fx, fy, cameraPose, true);

  //       // Get the map points at the current VSLAM step
  //       vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.getMap();
  //       for (auto point: mapPoints){
  //         cout << point->GetWorldPos() << endl;
  //       }
  //       cout << "****************************************************************" << endl;

  //       if (printTraj && !cameraPose.empty())
  //       {
  //         cout.precision(2);
  //         // cout << fixed << setw(8) << "X: " << -cameraPose.at<float>(2,3) << setw(8) << " Y: " << -cameraPose.at<float>(0,3) << setw(8) << " Z: " << -cameraPose.at<float>(1,3) << endl;
  //         cv::Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
  //         cv::Mat Twc = -Rwc * cameraPose.rowRange(0, 3).col(3);
  //         cout << fixed << setprecision(5) << setw(8) << "X: " << Twc.at<float>(2) << setw(8) << " Y: " << -Twc.at<float>(0) << setw(8) << " Z: " << -Twc.at<float>(1) << endl;
  //         // cout << cameraPose << endl;
  //         // cout << covMat << endl;
  //       }

  //       // Saving files
  //       if (saveFile) {
  //         char filename_ir_[50] = "./infrared/ir_";
  //         char *filename_ir = &filename_ir_[0];
  //         strcat(filename_ir, to_string(realsense.getIRLeftTimestamp()).c_str());
  //         strcat(filename_ir, ".jpg");
  //         imwrite(filename_ir, irMatrix);

  //         // depthMatrix.convertTo(depthMatrix, CV_8UC1, 15 / 256.0);

  //         char filename_depth_[50] = "./depth/depth_";
  //         char *filename_depth = &filename_depth_[0];
  //         strcat(filename_depth, to_string(realsense.getIRLeftTimestamp()).c_str());
  //         strcat(filename_depth, ".png");
  //         imwrite(filename_depth, depthMatrix);
  //       }
  //     }

  //     int key = waitKey(10);
  //     // Stop SLAM when Spacebar is pressed or if the map changed (so a loop has been closed)
  //     if( key == 32 || (SLAM.MapChanged() && autoclose)) {
  //       cout << "Loop closed ==> shutting down SLAM" << endl;
  //       break;
  //     }
  //   }

  //   // Stop all threads
  //   SLAM.Shutdown();

  //   // Save camera trajectory
  //   SLAM.SaveTrajectory("CameraTrajectory.dat");
  //   // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");
  // }
  // catch(exception& ex) {
  //   cout << ex.what() << endl;
  // }

  return 0;
}