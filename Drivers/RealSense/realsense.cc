#include "realsense.h"

// Constructor
RealSense::RealSense()
{
  initialize();
}

// Destructor
RealSense::~RealSense()
{
  finalize();
}

// Processing
void RealSense::run()
{
  // Main Loop
  while( true ) {
    update();
    draw();
    show();

    // Key Check
    const int32_t key = cv::waitKey( 10 );
    if( key == 'q' ){
      break;
    }
  }
}

// Public function for updating
void RealSense::updateAlign()
{
  update();
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
void RealSense::initialize()
{
  cv::setUseOptimized(true);
  initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
  // Set Device Config
  rs2::config config;
  config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
  config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );

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
void RealSense::update()
{
  updateFrame();
  updateColor();
  updateDepth();
}

// Update Frame
inline void RealSense::updateFrame()
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
inline void RealSense::updateColor()
{
  color_frame = aligned_frameset.get_color_frame();

  // Retrive Frame Information
  color_width = color_frame.as<rs2::video_frame>().get_width();
  color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepth()
{
  depth_frame = aligned_frameset.get_depth_frame();

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
