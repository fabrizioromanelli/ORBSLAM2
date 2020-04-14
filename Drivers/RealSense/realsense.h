#ifndef __REALSENSE__
#define __REALSENSE__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSense
{
public:
  // These enums are for setting the RealSense modalities.
  // RGBD - Uses RGB camera and Depth camera (not aligned, not synchronized)
  // IRD  - Uses Infrared Left camera and Depth camera (aligned, synchronized)
  enum sModality { RGBD, IRD };

  // Constructor
  RealSense(const sModality modality);

  // Constructor with maximum delta timeframes as an input
  RealSense(const sModality modality, double maximumDeltaTimeframes);

  // Destructor
  ~RealSense();

  // Process
  void run();

  // Operations with frame timestamps
  rs2_time_t getRGBTimestamp();
  rs2_time_t getDepthTimestamp();
  rs2_time_t getIRLeftTimestamp();
  rs2_time_t getTemporalFrameDisplacement();
  rs2_time_t getAverageTimestamp();

  bool isValidAlignedFrame();

  // Get frame matrices
  cv::Mat getColorMatrix();
  cv::Mat getDepthMatrix();

private:
  // RealSense
  rs2::pipeline pipeline;
  rs2::pipeline_profile pipeline_profile;
  rs2::frameset aligned_frameset;

  // Color Buffer
  rs2::frame color_frame;
  cv::Mat color_mat;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps = 30;

  // Infrared Left Buffer
  rs2::frame ir_left_frame;
  cv::Mat ir_left_mat;
  uint32_t ir_left_width = 640;
  uint32_t ir_left_height = 480;
  uint32_t ir_left_fps = 30;

  // Infrared Right Buffer
  rs2::frame ir_right_frame;
  cv::Mat ir_right_mat;
  uint32_t ir_right_width = 640;
  uint32_t ir_right_height = 480;
  uint32_t ir_right_fps = 30;

  // Depth Buffer
  rs2::frame depth_frame;
  cv::Mat depth_mat;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;

  // Warmup frames
  uint32_t warm_up_frames = 30;

  // Frameset
  rs2::frameset frameset;

  // Error
  rs2_error * e = 0;

  // Maximum delta between RGB and Depth image timeframes (time in ms)
  rs2_time_t maxDeltaTimeframes;
  rs2_time_t MIN_DELTA_TIMEFRAMES_THRESHOLD = 20;

  // Sensor modality
  sModality sensorModality;

  enum irCamera { IR_LEFT = 1, IR_RIGHT = 2};

private:
  // Initialize
  void initialize(rs2_time_t _maxDeltaTimeframes);

  // Initialize Sensor
  inline void initializeSensor();

  // Finalize
  void finalize();

  // Updates for aligned RGBD frames
  // Update Data
  void updateRGBD();

  // Update Frame
  inline void updateFrameAlign();

  // Update Color
  inline void updateColorRGBD();

  // Update Depth
  inline void updateDepthRGBD();

  // Updates for IRD frames
  void updateIRD();

  // Update Frame
  inline void updateFrame();

  // Update IR (Left)
  inline void updateInfraredIRD();

  // Update Depth
  inline void updateDepthIRD();

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