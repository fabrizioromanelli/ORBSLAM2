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

// Initialize
void RealSense::initialize()
{
  cv::setUseOptimized( true );
  initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
  // Set Device Config
  rs2::config config;
  config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
  config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );

  pipeline_profile = pipeline.start( config );
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
  rs2::frameset frameset = pipeline.wait_for_frames();

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
