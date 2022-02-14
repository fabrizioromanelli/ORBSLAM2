#ifndef __REALSENSE__
#define __REALSENSE__

#include <thread>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSense
{
public:
  // These enums are for setting the RealSense modalities.
  // RGBD - Uses RGB camera and Depth camera (not aligned, not synchronized)
  // IRD  - Uses Infrared Left camera and Depth camera (aligned, synchronized)
  // IRL  - Uses Infrared Left camera
  // IRR  - Uses Infrared Right camera
  // MULTI - Uses Infrared Left camera and Depth camera from D435i and pose from T265
  enum sModality { RGBD, IRD, IRL, IRR, MULTI };

private:
  // Sensor modality
  sModality sensorModality;

  // RealSense
  rs2::pipeline pipeline;
  std::vector<rs2::pipeline> pipelines; // for multicamera
  rs2::pipeline_profile pipeline_profile;
  rs2::frameset aligned_frameset;
  rs2::device realSense_device;

  // Color Buffer
  rs2::frame color_frame;
  cv::Mat color_mat;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps;

  // Infrared Left Buffer
  rs2::frame ir_left_frame;
  cv::Mat ir_left_mat;
  uint32_t ir_left_width = 640;
  uint32_t ir_left_height = 480;
  uint32_t ir_left_fps;

  // Infrared Right Buffer
  rs2::frame ir_right_frame;
  cv::Mat ir_right_mat;
  uint32_t ir_right_width = 640;
  uint32_t ir_right_height = 480;
  uint32_t ir_right_fps;

  // Depth Buffer
  rs2::frame depth_frame;
  cv::Mat depth_mat;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps;

  // Pose buffer
  rs2_pose pose;

  // Warmup frames
  uint32_t warm_up_frames = 30;

  // Framesets
  rs2::frameset frameset, frameset2;

  // Error
  rs2_error * e = 0;

  // Maximum delta between RGB and Depth image timeframes (time in ms)
  rs2_time_t maxDeltaTimeframes;
  rs2_time_t MIN_DELTA_TIMEFRAMES_THRESHOLD = 20;

  enum irCamera { IR_LEFT = 1, IR_RIGHT = 2 };
  enum frameCamera { D435I = 0, T265 = 1 };

  // Capture serial numbers before opening streaming
  std::vector<std::string> serials, names;

  std::vector<rs2::config> config;
public:
  // Constructor
  RealSense(const sModality);

  // Constructor with maximum delta timeframes as an input
  RealSense(const sModality, double);

  // Constructor with fps as an input
  RealSense(const sModality, uint32_t);

  // Destructor
  ~RealSense();

  // Process
  void run();

  // Operations with frame timestamps
  rs2_time_t getRGBTimestamp();
  rs2_time_t getDepthTimestamp();
  rs2_time_t getIRLeftTimestamp();
  rs2_time_t getPoseTimestamp();
  rs2_time_t getTemporalFrameDisplacement();
  rs2_time_t getAverageTimestamp();

  bool isValidAlignedFrame();

  // Get frame matrices
  cv::Mat getColorMatrix();
  cv::Mat getDepthMatrix();
  cv::Mat getIRLeftMatrix();
  cv::Mat getIRRightMatrix();

  // Get pose
  rs2_pose getPose();

  // Get raw frames
  rs2::frame getColorFrame();
  rs2::frame getDepthFrame();
  rs2::frame getIRLeftFrame();
  rs2::frame getIRRightFrame();

  // Reset pose tracking
  void resetPoseTrack();

  // Control laser projector
  void enableLaser(float);
  void disableLaser();

private:
  // Initialize
  void initialize(rs2_time_t);
  void initializeMulti(rs2_time_t);

  // Initialize Sensor
  inline void initializeSensor();
  inline void initializeSensors();

  // Finalize
  void finalize();

  // Updates for aligned RGBD frames
  // Update Data
  void updateRGBD();

  // Updates for IRD frames
  void updateIRD();

  // Updates for IR Left and Right frames
  void updateIRL();
  void updateIRR();

  // Updates for multi frames
  void updateMULTI();

  // Update pose frame
  inline void updatePose();

  // Update Frame
  inline void updateFrame();

  // Update Color
  inline void updateColor();

  // Update Depth
  inline void updateDepth();

  // Update IR (Left)
  inline void updateInfraredIRLeft();

  // Update IR (Right)
  inline void updateInfraredIRRight();

  // Draw Data
  void draw();

  // Draw Color
  inline void drawColor();

  // Draw Depth
  inline void drawDepth();

  // Show Data
  void show();

  // Show Color
  inline void showColor();

  // Show Depth
  inline void showDepth();
};

#endif // __REALSENSE__