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
#include <sys/stat.h>

#include "UwbApi.h"
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define VSLAM_FREQUENCY 30.0 // Hz
// #define DEBUG
// #define NO_UWB

#define NODENUMBER 3
const node_id_t uwb_master_id = 0x0E9E;
const node_id_t uwb_slave_ids[NODENUMBER] = {0x0C16, 0x45BB, 0x4890};
vector<double> uwbTimestamps;
vector<vector<uint16_t>> uwbReadings;
chrono::steady_clock::time_point tUwb;

void saveUWBreadings(const string &, vector<double>, vector<vector<uint16_t>>);
void saveCameraCovariances(const string &, vector<cv::Mat>);
void saveCameraPositions(const string &, vector<double>, vector<cv::Mat>);
Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &);
std::vector<float> toQuaternion(const cv::Mat &);

static void distanceFromUWBs(rall_descr_t* rall) {
  multi_range_with(uwb_master_id, uwb_slave_ids, NODENUMBER);

  vector<uint16_t> tmp;
  for(int i = 0; i < NODENUMBER; i++)
    tmp.push_back(rall->distances[i]);

  uwbReadings.push_back(tmp);
  auto nanoseconds_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  double millis = (double)nanoseconds_since_epoch / 1000000.0;
  uwbTimestamps.push_back(millis);
}


int main(int argc, char **argv)
{
  if(argc != 8)
  {
    cerr << endl << "Usage: ./uwb" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl
                 << "         specify device file name [e.g. /dev/ttyACM0]" << endl
                 << "Example: ./Test/Live/UWB/uwb Vocabulary/orb_mur.fbow Config/RealSense-D435i-IRD.yaml OFF OFF OFF OFF /dev/ttyACM0" << endl;
    return 1;
  }

  try {
    // Initialize sensors
    RealSense realsense(RealSense::IRD, (uint32_t)VSLAM_FREQUENCY);

    // Clone parameters from command line
    bool display = false;
    string displayS = string(argv[3]);
    if(displayS.compare("ON") == 0)
      display = true;

    bool saveFile = false;
    string saveFileS = string(argv[4]);
    if(saveFileS.compare("ON") == 0)
      saveFile = true;

    bool autoclose = false;
    string autocloseS = string(argv[5]);
    if(autocloseS.compare("ON") == 0)
      autoclose = true;

    bool printTraj = false;
    string printTrajS = string(argv[6]);
    if(printTrajS.compare("ON") == 0)
      printTraj = true;

    #ifndef NO_UWB
      init(argv[7]);
      sleep(1);
      // Registering callback to get distances from UWBs
      register_rall_cb(distanceFromUWBs);
      multi_range_with(uwb_master_id, uwb_slave_ids, NODENUMBER);
    #endif

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, display, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    chrono::steady_clock::time_point tSlam_start;
    chrono::steady_clock::time_point tSlam_end;
    chrono::steady_clock::time_point tSlam_prev  = chrono::steady_clock::now();

    vector<double> cameraTimestamps;
    vector<cv::Mat> covarianceMatrices, invCovarianceMatrices, cameraPoses;

    float fx = 379.895904541016 / 640; // expressed in meters
    float fy = 379.895904541016 / 480; // expressed in meters

    // Main loop
    for(;;)
    {
      tSlam_start = chrono::steady_clock::now();
      #ifdef DEBUG
        cout << "VSLAM frequency: " << 1/chrono::duration_cast<chrono::duration<double> >(tSlam_start - tSlam_prev).count() << "Hz" << endl;
      #endif
      tSlam_prev = chrono::steady_clock::now();

      realsense.run();
      // cout << fixed << setw(11) << setprecision(6) << "Timestamp SLAM  : " << realsense.getIRLeftTimestamp() << endl;
      cv::Mat irMatrix    = realsense.getIRLeftMatrix();
      cv::Mat depthMatrix = realsense.getDepthMatrix();

      // Pass the IR Left and Depth images to the SLAM system
      cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, realsense.getIRLeftTimestamp());
      covarianceMatrices.push_back(SLAM.GetCurrentCovarianceMatrix(fx, fy, cameraPose, false));
      invCovarianceMatrices.push_back(SLAM.GetCurrentCovarianceMatrix(fx, fy, cameraPose, true));

      if (!cameraPose.empty())
      {
        cameraPoses.push_back(cameraPose);
        cameraTimestamps.push_back(realsense.getIRLeftTimestamp());
      }

      tSlam_end = chrono::steady_clock::now();

      double ttrack = chrono::duration_cast<chrono::duration<double> >(tSlam_end - tSlam_start).count();

      #ifdef DEBUG
        cout << "Track frequency: " << 1/ttrack << "Hz" << endl;
      #endif

      if (printTraj && !cameraPose.empty())
        cout << "Camera position" << cameraPose << endl;

      // Saving files
      if (saveFile) {
        char filename_ir_[50] = "./infrared/ir_";
        char *filename_ir = &filename_ir_[0];
        strcat(filename_ir, to_string(realsense.getIRLeftTimestamp()).c_str());
        strcat(filename_ir, ".jpg");
        imwrite(filename_ir, irMatrix);

        // depthMatrix.convertTo(depthMatrix, CV_8UC1, 15 / 256.0);

        char filename_depth_[50] = "./depth/depth_";
        char *filename_depth = &filename_depth_[0];
        strcat(filename_depth, to_string(realsense.getIRLeftTimestamp()).c_str());
        strcat(filename_depth, ".png");
        imwrite(filename_depth, depthMatrix);
        }

      // Stop SLAM if the map changed (so a loop has been closed)
      if(SLAM.MapChanged() && autoclose) {
        cout << "Loop closed ==> shutting down SLAM" << endl;
        break;
      }

      // Sleep according to the VSLAM frequency
      // if(1/ttrack > VSLAM_FREQUENCY)
      //   this_thread::sleep_for(chrono::microseconds(static_cast<size_t>((1/VSLAM_FREQUENCY-ttrack)*1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.dat");

    // Save camera positions pre-loop closure
    saveCameraPositions("CameraTrajectory-preLoopClosure.dat", cameraTimestamps, cameraPoses);

    // Save UWB readings
    saveUWBreadings("UWBReadings.dat", uwbTimestamps, uwbReadings);

    // Save covariance matrices
    saveCameraCovariances("cMatrices.dat", covarianceMatrices);

    // Save inverse covariance matrices
    saveCameraCovariances("cMatricesInverse.dat", invCovarianceMatrices);
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

void saveUWBreadings(const string &filename, vector<double> timestamps, vector<vector<uint16_t>> readings)
{
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  vector<vector<uint16_t>>::iterator itReadings = readings.begin();

  for(vector<double>::iterator itTimestamps = timestamps.begin(); itTimestamps != timestamps.end(); itTimestamps++, itReadings++)
  {
    vector<uint16_t> _readings = *itReadings;
    f << setprecision(6) << *itTimestamps << " " <<  setprecision(9) << _readings.at(0) << " " << _readings.at(1) << " " << _readings.at(2) << endl;
  }
  f.close();
  cout << endl << "UWB readings saved!" << endl;
}

void saveCameraCovariances(const string &filename, vector<cv::Mat> cMatrices)
{
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for(vector<cv::Mat>::iterator itCMatrices = cMatrices.begin(); itCMatrices != cMatrices.end(); itCMatrices++)
  {
    cv::Mat tmp = *itCMatrices;
    f << setprecision(9);
    for (size_t i = 0; i < 6; i++)
      for (size_t j = 0; j < 6; j++)
        f << tmp.at<float>(i,j) << " ";

    f << endl;
  }
  f.close();
  cout << endl << "Covariance matrices saved!" << endl;
}

void saveCameraPositions(const string &filename, vector<double> timestamps, vector<cv::Mat> cPositions)
{
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  vector<cv::Mat>::iterator itPositions = cPositions.begin();

  for(vector<double>::iterator itTimestamps = timestamps.begin(); itTimestamps != timestamps.end(); itTimestamps++, itPositions++)
  {
    cv::Mat tmp = *itPositions;

    cv::Mat Rwc = tmp.rowRange(0,3).colRange(0,3).t();
    cv::Mat Twc = -Rwc*tmp.rowRange(0,3).col(3);

    vector<float> q = toQuaternion(Rwc);

    f << setprecision(6) << *itTimestamps << " " <<  setprecision(9) << Twc.at<float>(0) << " " << Twc.at<float>(1) << " " << Twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
  }
  f.close();
  cout << endl << "Camera position pre-loop closure saved!" << endl;
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}