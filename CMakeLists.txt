cmake_minimum_required(VERSION 3.0)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

OPTION(JETSON_BUILD "Jetson build is: " OFF)
IF(JETSON_BUILD)
    ADD_DEFINITIONS(-D__ANDROID__)
ENDIF(JETSON_BUILD)

OPTION(REALSENSE_BUILD "Realsense build is: " ON)
IF(REALSENSE_BUILD)
  set(REALSENSE RealSense)
ENDIF()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")

# Check C++14, C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14) # Added to let ORBSLAM2 compile in Ubuntu 22.04
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14/C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(OpenCV 4.5.1 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.5.1 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
IF(REALSENSE_BUILD)
  find_package(realsense2 REQUIRED)
ENDIF()

IF(REALSENSE_BUILD)
  include_directories(
          ${PROJECT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/Drivers/RealSense
          ${EIGEN3_INCLUDE_DIR}
          ${Pangolin_INCLUDE_DIRS})
ELSE()
  include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS})
ENDIF()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/HPose.cpp
        #src/QrCodeTracker.cc
        #src/QrCode.cc
        #src/ArucoCode.cc
        #src/ArucoCodeScanner.cc
        )

IF(REALSENSE_BUILD)
  target_link_libraries(${PROJECT_NAME}
          ${REALSENSE}
          ${OpenCV_LIBS}
          ${EIGEN3_LIBS}
          ${Pangolin_LIBRARIES}
          ${realsense2_LIBRARY}
          ${PROJECT_SOURCE_DIR}/Thirdparty/DLib/lib/libDLib.so
          ${PROJECT_SOURCE_DIR}/Thirdparty/fbow/lib/libfbow.so
          ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  #        ${PROJECT_SOURCE_DIR}/lib/libuwb.so
          )

  # Adding RealSense driver library
  add_library(${REALSENSE} SHARED Drivers/RealSense/realsense.cc)
ELSE()
  target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DLib/lib/libDLib.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/fbow/lib/libfbow.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  #        ${PROJECT_SOURCE_DIR}/lib/libuwb.so
  )
ENDIF()

IF(REALSENSE_BUILD)
  target_link_libraries(${REALSENSE} ${OpenCV_LIBS})
ENDIF()

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
  message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build Tests for Live cameras
IF(REALSENSE_BUILD)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/IRD)
  add_executable(realsense_live Test/Live/IRD/realsense.cc)
  target_link_libraries(realsense_live ${PROJECT_NAME})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/Multicam)
  add_executable(multicam_live Test/Live/Multicam/multicam.cc)
  target_link_libraries(multicam_live ${PROJECT_NAME})
ENDIF()
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/Monocular)
# add_executable(mono_camera Test/Live/Monocular/mono_camera.cc)
# target_link_libraries(mono_camera ${PROJECT_NAME})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/QRCode)
#add_executable(qrCode_live Test/Live/QRCode/qrCode.cc)
#target_link_libraries(qrCode_live ${PROJECT_NAME})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/ArucoCode)
#add_executable(arucoCode Test/Live/ArucoCode/arucoCode.cc)
#target_link_libraries(arucoCode ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/UWB)
# add_executable(uwb Test/Live/UWB/uwb.cc)
# target_link_libraries(uwb -lpthread ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Live/UWB)
# add_executable(uwb_bias Test/Live/UWB/uwb_bias.cc)
# target_link_libraries(uwb_bias -lpthread ${PROJECT_NAME})

# # Build Tests for Replay sequences
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Replay/IRD)
add_executable(realsense_replay Test/Replay/IRD/realsense.cc)
target_link_libraries(realsense_replay ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Replay/Monocular)
# add_executable(mono_euroc Test/Replay/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})
# add_executable(mono_kitti Test/Replay/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})
# add_executable(mono_tum Test/Replay/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Replay/RGBD)
# add_executable(RGBD-tum Test/Replay/RGBD/rgbd_tum.cc)
# target_link_libraries(RGBD-tum ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Replay/Stereo)
# add_executable(stereo_kitti Test/Replay/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})
add_executable(stereo_euroc Test/Replay/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})
add_executable(stereo_isl Test/Replay/Stereo/stereo_isl.cc)
target_link_libraries(stereo_isl ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test/Replay/QRCode)
# add_executable(qrCode_replay Test/Replay/QRCode/qrCode.cc)
# target_link_libraries(qrCode_replay ${PROJECT_NAME})

# #
# # Tools
# #
# # Benchmarks
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Benchmarks/Live/Monocular)
# add_executable(benchmark_mono_live Tools/Benchmarks/Live/Monocular/benchmark.cc)
# target_link_libraries(benchmark_mono_live ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Benchmarks/Replay/Monocular)
# add_executable(benchmark_mono_replay Tools/Benchmarks/Replay/Monocular/benchmark.cc)
# target_link_libraries(benchmark_mono_replay ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Benchmarks/Replay/RGBD)
# add_executable(benchmark_rgbd_replay Tools/Benchmarks/Replay/RGBD/benchmark.cc)
# target_link_libraries(benchmark_rgbd_replay ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Benchmarks/Replay/Stereo)
# add_executable(benchmark_stereo_replay Tools/Benchmarks/Replay/Stereo/benchmark.cc)
# target_link_libraries(benchmark_stereo_replay ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Benchmarks/Replay/IRD)
# add_executable(benchmark_ird_replay Tools/Benchmarks/Replay/IRD/benchmark.cc)
# target_link_libraries(benchmark_ird_replay ${PROJECT_NAME})

# # Camera Calibration
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Calibration/Monocular)
# add_executable(camera_calibration Tools/Calibration/Monocular/camera_calibration.cc)
# target_link_libraries(camera_calibration ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Calibration/Stereo)
# add_executable(camera_calibration_stereo Tools/Calibration/Stereo/camera_calibration_stereo.cc)
# target_link_libraries(camera_calibration_stereo ${PROJECT_NAME})

# # Grab single images
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Grab/Depth)
# add_executable(grab_realsense_depth Tools/Grab/Depth/grab_realsense_depth.cc)
# target_link_libraries(grab_realsense_depth ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Grab/Infrared)
# add_executable(grab_realsense_ir Tools/Grab/Infrared/grab_realsense_ir.cc)
# target_link_libraries(grab_realsense_ir ${PROJECT_NAME})
# # Here we unset the CMAKE_BUILD_TYPE because -O3 option causes a crash
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Grab/Monocular)
# add_executable(grab_monocular Tools/Grab/Monocular/grab_monocular.cc)
# target_compile_options(grab_monocular PRIVATE -O0)
# target_link_libraries(grab_monocular ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/Grab/RGB)
# add_executable(grab_realsense_rgb Tools/Grab/RGB/grab_realsense_rgb.cc)
# target_link_libraries(grab_realsense_rgb ${PROJECT_NAME})

# # Grab sequencess
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools/GrabSequence/IRD)
# add_executable(grab_sequence_realsense_ird Tools/GrabSequence/IRD/grab_sequence_realsense_ird.cc)
# target_link_libraries(grab_sequence_realsense_ird ${PROJECT_NAME})

# Install library and headers
install (TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "/usr/local/lib")

install(DIRECTORY "Thirdparty/fbow/lib/"
    DESTINATION "/usr/local/lib")

install(DIRECTORY "Thirdparty/g2o/lib/"
    DESTINATION "/usr/local/lib")

install(DIRECTORY "Thirdparty/DLib/lib/"
    DESTINATION "/usr/local/lib")

install(DIRECTORY "Thirdparty/fbow/"
    DESTINATION "/usr/local/include/Thirdparty/fbow"
    FILES_MATCHING
    PATTERN "*.h")

install(DIRECTORY "Thirdparty/DLib/include/"
    DESTINATION "/usr/local/include/Thirdparty/DLib")

install(DIRECTORY "Thirdparty/g2o/g2o/"
    DESTINATION "/usr/local/include/Thirdparty/g2o/g2o")

install(DIRECTORY "Thirdparty/g2o/"
    DESTINATION "/usr/local/include/Thirdparty/g2o"
    FILES_MATCHING
    PATTERN "*.h")

install(DIRECTORY "include/"
        DESTINATION "/usr/local/include/${PROJECT_NAME}"
        FILES_MATCHING
        PATTERN "*.h")

install(DIRECTORY "Config/"
        DESTINATION "/usr/local/share/${PROJECT_NAME}/Config")

install(DIRECTORY "Vocabulary/"
        DESTINATION "/usr/local/share/${PROJECT_NAME}/Vocabulary"
        FILES_MATCHING
        PATTERN "*.fbow")
