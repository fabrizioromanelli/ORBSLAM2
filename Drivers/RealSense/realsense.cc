#include "realsense.h"

// Constructor
RealSense::RealSense(const sModality modality):
sensorModality(modality), color_fps(30), ir_left_fps(30), ir_right_fps(30), depth_fps(30)
{
  if (modality == RGBD || modality == IRD || modality == IRL || modality == IRR) {
    rs2::config _config;
    config.reserve(1);
    config[D435I] = _config;
    initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
  }
  else if (modality == MULTI) {
    rs2::config _config, _config2;
    config.push_back(_config);
    config.push_back(_config2);
    initializeMulti(MIN_DELTA_TIMEFRAMES_THRESHOLD);
  }
}

// OBSOLETE CONSTRUCTOR
// Constructor with maximum delta timeframes as an input
RealSense::RealSense(const sModality modality, double maximumDeltaTimeframes):
sensorModality(modality), color_fps(30), ir_left_fps(30), ir_right_fps(30), depth_fps(30)
{
  if (maximumDeltaTimeframes > MIN_DELTA_TIMEFRAMES_THRESHOLD)
    initialize(maximumDeltaTimeframes);
  else
  {
    std::cerr << "Maximum delta between timeframes is too high. Reduced to " << MIN_DELTA_TIMEFRAMES_THRESHOLD << "." << std::endl;
    initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
  }
}

// OBSOLETE CONSTRUCTOR
RealSense::RealSense(const sModality modality, uint32_t fps):
sensorModality(modality), color_fps(fps), ir_left_fps(fps), ir_right_fps(fps), depth_fps(fps)
{
  initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
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
    case IRL:
      updateIRL();
      break;
    case IRR:
      updateIRR();
      break;
    case MULTI:
      updateMULTI();
      break;
    default:
      break;
  }
}

rs2_time_t RealSense::getRGBTimestamp()
{
  if (sensorModality == RGBD) {
    // Get each frame
    rs2::frame cFrame  = aligned_frameset.get_color_frame();
    rs2_frame * frameP = cFrame.get();

    // Get frame timestamp
    return(rs2_get_frame_timestamp(frameP, &e));
  } else {
    return(-1);
  }
}

rs2_time_t RealSense::getDepthTimestamp()
{
  rs2::frame cFrame;
  switch (sensorModality)
  {
    case RGBD:
      cFrame  = aligned_frameset.get_depth_frame();
      break;
    case IRD:
      cFrame  = frameset.get_depth_frame();
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }

  rs2_frame * frameP = cFrame.get();

  // Get frame timestamp
  return(rs2_get_frame_timestamp(frameP, &e));
}

rs2_time_t RealSense::getIRLeftTimestamp()
{
  if ((sensorModality == IRD) || (sensorModality == IRL) || (sensorModality == MULTI)) {
    // Get each frame
    rs2::frame cFrame  = frameset.get_infrared_frame(IR_LEFT);
    rs2_frame * frameP = cFrame.get();

    // Get frame timestamp
    return(rs2_get_frame_timestamp(frameP, &e));
  } else {
    return(-1);
  }
}

rs2_time_t RealSense::getPoseTimestamp()
{
  if (sensorModality == MULTI) {
    // Get each frame
    auto cFrame = frameset2.get_pose_frame();
    rs2_frame * frameP = cFrame.get();

    // Get frame timestamp
    return(rs2_get_frame_timestamp(frameP, &e));
  } else {
    return(-1);
  }
}

// This function gets the temporal displacement between
// the RGB and Depth frames (their delta).
rs2_time_t RealSense::getTemporalFrameDisplacement()
{
  switch (sensorModality)
  {
    case RGBD:
      return(fabs(getRGBTimestamp()-getDepthTimestamp()));
      break;
    case IRD:
      return(fabs(getIRLeftTimestamp()-getDepthTimestamp()));
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }
  return(0);
}

// This function gets the average timestamp between
// the RGB and Depth frames.
rs2_time_t RealSense::getAverageTimestamp()
{
  switch (sensorModality)
  {
    case RGBD:
      return((getRGBTimestamp()+getDepthTimestamp())/2.0);
      break;
    case IRD:
      return((getIRLeftTimestamp()+getDepthTimestamp())/2.0);
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }
  return(0);
}

bool RealSense::isValidAlignedFrame()
{
  if (sensorModality == RGBD) {
    if (getTemporalFrameDisplacement() >= maxDeltaTimeframes)
      return(false);
    else
      return(true);
  } else {
    return(true);
  }
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
  cv::Mat depth(cv::Size(depth_width, depth_height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
  return(depth);
}

// Get IR left matrix
cv::Mat RealSense::getIRLeftMatrix()
{
  cv::Mat ir_left(cv::Size(ir_left_width, ir_left_height), CV_8UC1, (void*)ir_left_frame.get_data(), cv::Mat::AUTO_STEP);
  return(ir_left);
}

// Get IR right matrix
cv::Mat RealSense::getIRRightMatrix()
{
  cv::Mat ir_right(cv::Size(ir_right_width, ir_right_height), CV_8UC1, (void*)ir_right_frame.get_data(), cv::Mat::AUTO_STEP);
  return(ir_right);
}

rs2_pose RealSense::getPose()
{
  rs2_pose tmp = pose;
  tmp.translation.x = -pose.translation.z;
  tmp.translation.y = -pose.translation.x;
  tmp.translation.z = pose.translation.y;
  tmp.rotation.w    = pose.rotation.w;
  tmp.rotation.x    = pose.rotation.z;
  tmp.rotation.y    = -pose.rotation.x;
  tmp.rotation.z    = pose.rotation.y;

  return(tmp);
}

// Get color frame
rs2::frame RealSense::getColorFrame()
{
  return(color_frame);
}

// Get depth frame
rs2::frame RealSense::getDepthFrame()
{
  return(depth_frame);
}

// Get IR left frame
rs2::frame RealSense::getIRLeftFrame()
{
  return(ir_left_frame);
}

// Get IR right frame
rs2::frame RealSense::getIRRightFrame()
{
  return(ir_right_frame);
}

// Initialize
void RealSense::initialize(rs2_time_t _maxDeltaTimeFrames)
{
  maxDeltaTimeframes = _maxDeltaTimeFrames;
  cv::setUseOptimized(true);
  initializeSensor();
}

// Initialize
void RealSense::initializeMulti(rs2_time_t _maxDeltaTimeFrames)
{
  maxDeltaTimeframes = _maxDeltaTimeFrames;
  cv::setUseOptimized(true);
  initializeSensors();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
  switch (sensorModality)
  {
    case RGBD:
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    case IRD:
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      // The following is just needed to get the correct baseline information, it will be then disabled.
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    case IRL:
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      break;
    case IRR:
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      break;
    default:
      std::cerr << "Invalid modality selected" << std::endl;
      break;
  }

  pipeline_profile = pipeline.start(config[D435I]);
  realSense_device = pipeline_profile.get_device();

  // Refer to: https://github.com/raulmur/ORB_SLAM2/issues/259
  if (sensorModality == IRD)
  {
    auto depth_sensor = realSense_device.first<rs2::depth_sensor>();
    auto depth_stream = pipeline_profile.get_stream(RS2_STREAM_DEPTH);
    auto ir_stream    = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2);

    const float scale = depth_sensor.get_depth_scale();
    rs2_intrinsics intrinsics = pipeline_profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
    rs2_extrinsics extrinsics = depth_stream.get_extrinsics_to(ir_stream);

    std::stringstream ss;
    ss << "    " << std::left << std::setw(31) << "Width"      << ": " << intrinsics.width << std::endl <<
          "    " << std::left << std::setw(31) << "Height"     << ": " << intrinsics.height << std::endl <<
          "    " << std::left << std::setw(31) << "Distortion" << ": " << rs2_distortion_to_string(intrinsics.model) << std::endl <<
          "    " << std::left << std::setw(31) << "Baseline"   << ": " << std::setprecision(15) << extrinsics.translation[0] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.fx"  << ": " << std::setprecision(15) << intrinsics.fx << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.fy"  << ": " << std::setprecision(15) << intrinsics.fy << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.cx"  << ": " << std::setprecision(15) << intrinsics.ppx << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.cy"  << ": " << std::setprecision(15) << intrinsics.ppy << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k1"  << ": " << std::setprecision(15) << intrinsics.coeffs[0] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k2"  << ": " << std::setprecision(15) << intrinsics.coeffs[1] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.p1"  << ": " << std::setprecision(15) << intrinsics.coeffs[2] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.p1"  << ": " << std::setprecision(15) << intrinsics.coeffs[3] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k3"  << ": " << std::setprecision(15) << intrinsics.coeffs[4] << std::endl <<
          "    " << std::left << std::setw(31) << "DepthMapFactor" << ": " << 1/scale << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.bf"  << ": " << fabs(extrinsics.translation[0]*intrinsics.fx) << std::endl;

    std::cout << ss.str() << std::endl;
    pipeline.stop();
    config[D435I].disable_stream(RS2_STREAM_INFRARED, 2);
    pipeline_profile = pipeline.start(config[D435I]);
    realSense_device = pipeline_profile.get_device();
  }

  // Disabled by default the laser projector
  disableLaser();

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  for (uint32_t i = 0; i < warm_up_frames; i++)
  {
    // Wait for all configured streams to produce a frame
    frameset = pipeline.wait_for_frames();
  }
}

// Initialize Sensors for multicamera
inline void RealSense::initializeSensors()
{
  rs2::context ctx; // Create librealsense context for managing devices
  pipelines.reserve(2);

  // Capture serial numbers before opening streaming
  for (auto&& dev : ctx.query_devices())
  {
    serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    names.push_back(dev.get_info(RS2_CAMERA_INFO_NAME));
  }

  size_t i = 0;
  // Start a streaming pipe per each connected device
  for (auto&& serial : serials)
  {
    rs2::pipeline pipe(ctx);

    std::string t265("T265");
    if (names[i].find(t265) != std::string::npos)
    {
      config[T265].enable_device(serial);
      pipe.start(config[T265]);
      pipelines[T265] = pipe;
      // Camera warmup - dropping several first frames to let auto-exposure stabilize
      for (uint32_t i = 0; i < warm_up_frames; i++)
      {
        // Wait for all configured streams to produce a frame
        frameset2 = pipe.wait_for_frames();
      }
      std::cout << "Intel RealSense T265 initialized." << std::endl;
    }
    else // should be a D435i
    {
      config[D435I].enable_device(serial);
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      config[D435I].enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );

      pipeline_profile = pipe.start(config[D435I]);
      realSense_device = pipeline_profile.get_device();
      pipelines[D435I] = pipe;

      // Must do this twice every time you initialize a D400!
      //auto advanced = realSense_device.as<rs400::advanced_mode>();
      //advanced.toggle_advanced_mode(true);
      //STHdad changeSTDdad = advanced.get_hdad();
      //changeSTDdad.ignoreSAD = 1;
      //advanced.set_hdad(changeSTDdad);

      // Disabled by default the laser projector
      disableLaser();

      // Camera warmup - dropping several first frames to let auto-exposure stabilize
      for (uint32_t i = 0; i < warm_up_frames; i++)
      {
        // Wait for all configured streams to produce a frame
        frameset = pipe.wait_for_frames();
      }
      std::cout << "Intel RealSense D435i initialized." << std::endl;
    }

    i++;
  }
}

void RealSense::resetPoseTrack()
{
  pipelines[T265].stop();
  std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
  pipelines[T265].start(config[T265]);
}

void RealSense::enableLaser(float power)
{
  auto depth_sensor = realSense_device.first<rs2::depth_sensor>();

  if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, power);
}

void RealSense::disableLaser()
{
  auto depth_sensor = realSense_device.first<rs2::depth_sensor>();

  if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
}

// Finalize
void RealSense::finalize()
{
  cv::destroyAllWindows();
  if (sensorModality != MULTI) {
    pipeline.stop();
  } else {
    pipelines[D435I].stop();
    pipelines[T265].stop();
  }
}

// Update Data
void RealSense::updateRGBD()
{
  updateFrame();
  updateColor();
  updateDepth();
}

void RealSense::updateIRD()
{
  updateFrame();
  updateInfraredIRLeft();
  updateDepth();
}

void RealSense::updateIRL()
{
  updateFrame();
  updateInfraredIRLeft();
}

void RealSense::updateIRR()
{
  updateFrame();
  updateInfraredIRRight();
}

void RealSense::updateMULTI()
{
  updateFrame();
  updateInfraredIRLeft();
  updateDepth();
  updatePose();
}

// Update Frame
inline void RealSense::updateFrame()
{
  if (sensorModality != MULTI) {
    frameset = pipeline.wait_for_frames();

    if (sensorModality == RGBD) {
      // Retrieve Aligned Frame
      rs2::align align( rs2_stream::RS2_STREAM_COLOR );
      aligned_frameset = align.process( frameset );
      if( !aligned_frameset.size() ){
        return;
      }
    }
  } else {
    frameset  = pipelines[D435I].wait_for_frames();
    frameset2 = pipelines[T265].wait_for_frames();
  }
}

// Update Color
inline void RealSense::updateColor()
{
  if (sensorModality == RGBD)
    color_frame = aligned_frameset.get_color_frame();
  else
    color_frame = frameset.get_color_frame();

  // Retrieve Frame Information
  color_width = color_frame.as<rs2::video_frame>().get_width();
  color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepth()
{
  if (sensorModality == RGBD)
    depth_frame = aligned_frameset.get_depth_frame();
  else
    depth_frame = frameset.get_depth_frame();

  // Retrieve Frame Information
  depth_width = depth_frame.as<rs2::video_frame>().get_width();
  depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Update Infrared (Left)
inline void RealSense::updateInfraredIRLeft()
{
  ir_left_frame  = frameset.get_infrared_frame(IR_LEFT);

  // Retrieve Frame Information
  ir_left_width  = ir_left_frame.as<rs2::video_frame>().get_width();
  ir_left_height = ir_left_frame.as<rs2::video_frame>().get_height();
}

// Update Infrared (Right)
inline void RealSense::updateInfraredIRRight()
{
  ir_right_frame  = frameset.get_infrared_frame(IR_RIGHT);

  // Retrive Frame Information
  ir_right_width  = ir_right_frame.as<rs2::video_frame>().get_width();
  ir_right_height = ir_right_frame.as<rs2::video_frame>().get_height();
}

// Update Pose
inline void RealSense::updatePose()
{
  auto pose_frame = frameset2.get_pose_frame();
  pose = pose_frame.get_pose_data();
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
