#include "realsense.h"

// Constructor
RealSense::RealSense(const sModality modality)
{
  initialize(modality, MIN_DELTA_TIMEFRAMES_THRESHOLD);
}

// Constructor with maximum delta timeframes as an input
RealSense::RealSense(const sModality modality, double maximumDeltaTimeframes)
{
  if (maximumDeltaTimeframes > MIN_DELTA_TIMEFRAMES_THRESHOLD)
    initialize(modality, maximumDeltaTimeframes);
  else
  {
    std::cerr << "Maximum delta between timeframes is too high. Reduced to " << MIN_DELTA_TIMEFRAMES_THRESHOLD << "." << std::endl;
    initialize(modality, MIN_DELTA_TIMEFRAMES_THRESHOLD);
  }
}

// Destructor
RealSense::~RealSense()
{
  finalize();
}

// Public function for processing and updating sensor
void RealSense::run()
{
  switch (sensorModality)
  {
    case RGBD:
      updateRGBD();
      break;
    case IRD:
      updateIRD();
      break;
    default:
      break;
  }
}

rs2_time_t RealSense::getRGBTimestamp()
{
  // Get each frame
  rs2::frame cFrame  = aligned_frameset.get_color_frame();
  rs2_frame * frameP = cFrame.get();

  // Get frame timestamp
  return(rs2_get_frame_timestamp(frameP, &e));
}

rs2_time_t RealSense::getDepthTimestamp()
{
  // Get each frame
  rs2::frame cFrame  = aligned_frameset.get_depth_frame();
  rs2_frame * frameP = cFrame.get();

  // Get frame timestamp
  return(rs2_get_frame_timestamp(frameP, &e));
}

// This function gets the temporal displacement between
// the RGB and Depth frames (their delta).
rs2_time_t RealSense::getTemporalFrameDisplacement()
{
  return(fabs(getRGBTimestamp()-getDepthTimestamp()));
}

// This function gets the average timestamp between
// the RGB and Depth frames.
rs2_time_t RealSense::getAverageTimestamp()
{
  return((getRGBTimestamp()+getDepthTimestamp())/2.0);
}

bool RealSense::isValidAlignedFrame()
{
  if (getTemporalFrameDisplacement() >= maxDeltaTimeframes)
    return(false);
  else
    return(true);
}

// Get color matrix
cv::Mat RealSense::getColorMatrix()
{
  cv::Mat color(cv::Size(color_width, color_height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
  return(color);
}

// Get depth matrix
cv::Mat RealSense::getDepthMatrix()
{
  cv::Mat depth(cv::Size(depth_width, depth_height), CV_16SC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
  return(depth);
}

// Initialize
void RealSense::initialize(const sModality modality, rs2_time_t _maxDeltaTimeFrames)
{
  maxDeltaTimeframes = _maxDeltaTimeFrames;
  cv::setUseOptimized(true);
  initializeSensor(modality);
}

// Initialize Sensor
inline void RealSense::initializeSensor(const sModality modality)
{
  // Set Device Config
  rs2::config config;

  switch (modality)
  {
    case RGBD:
      config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    case IRD:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, 1, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    default:
      std::cerr << "Invalid modality selected" << std::endl;
      break;
  }

  // Set the sensor modality as from constructor
  sensorModality = modality;

  pipeline_profile = pipeline.start(config);

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  for (uint32_t i = 0; i < warm_up_frames; i++)
  {
    // Wait for all configured streams to produce a frame
    frameset = pipeline.wait_for_frames();
  }
}

// Finalize
void RealSense::finalize()
{
  cv::destroyAllWindows();
  pipeline.stop();
}

// Update Data
void RealSense::updateRGBD()
{
  updateFrameAlign();
  updateColorRGBD();
  updateDepthRGBD();
}

// Update Frame
inline void RealSense::updateFrameAlign()
{
  frameset = pipeline.wait_for_frames();

  // Retrieve Aligned Frame
  rs2::align align( rs2_stream::RS2_STREAM_COLOR );
  aligned_frameset = align.process( frameset );
  if( !aligned_frameset.size() ){
    return;
  }
}

// Update Color
inline void RealSense::updateColorRGBD()
{
  color_frame = aligned_frameset.get_color_frame();

  // Retrive Frame Information
  color_width = color_frame.as<rs2::video_frame>().get_width();
  color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepthRGBD()
{
  depth_frame = aligned_frameset.get_depth_frame();

  // Retrive Frame Information
  depth_width = depth_frame.as<rs2::video_frame>().get_width();
  depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Update Data
void RealSense::updateIRD()
{
  updateFrame();
  updateInfraredIRD();
  updateDepthIRD();
}

// Update Frame
inline void RealSense::updateFrame()
{
  frameset = pipeline.wait_for_frames();
}

// Update Infrared (Left)
inline void RealSense::updateInfraredIRD()
{
  ir_left_frame  = frameset.get_infrared_frame(1);

  // Retrive Frame Information
  ir_left_width  = ir_left_frame.as<rs2::video_frame>().get_width();
  ir_left_height = ir_left_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepthIRD()
{
  depth_frame = frameset.get_depth_frame();

  // Retrive Frame Information
  depth_width = depth_frame.as<rs2::video_frame>().get_width();
  depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Draw Data
void RealSense::draw()
{
  drawColor();
  drawDepth();
}

// Draw Color
inline void RealSense::drawColor()
{
  color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) );
}

// Draw Depth
inline void RealSense::drawDepth()
{
  depth_mat = cv::Mat( depth_height, depth_width, CV_16SC1, const_cast<void*>( depth_frame.get_data() ) );
}

// Show Data
void RealSense::show()
{
  showColor();
  showDepth();
}

// Show Color
inline void RealSense::showColor()
{
  if( color_mat.empty() ){
    return;
  }

  cv::imshow( "Color", color_mat );
}

// Show Depth
inline void RealSense::showDepth()
{
  if( depth_mat.empty() ){
    return;
  }

  // Scaling
  cv::Mat scale_mat;
  depth_mat.convertTo( scale_mat, CV_8U, -255.0 / 10000.0, 255.0 ); // 0-10000 -> 255(white)-0(black)
  //depth_mat.convertTo( scale_mat, CV_8U, 255.0 / 10000.0, 0.0 ); // 0-10000 -> 0(black)-255(white)

  cv::imshow( "Depth", scale_mat );
}
