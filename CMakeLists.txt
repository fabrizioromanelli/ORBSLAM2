cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2.0 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.2.0 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

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
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DLib/lib/libDLib.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/fbow/lib/libfbow.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        )

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

# Build benchmarks
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Benchmarks/fromMonocularFile)
add_executable(benchmark_mono_file Benchmarks/fromMonocularFile/benchmark.cc)
target_link_libraries(benchmark_mono_file ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Benchmarks/fromRGBDFile)
add_executable(benchmark_rgbd_file Benchmarks/fromRGBDFile/benchmark.cc)
target_link_libraries(benchmark_rgbd_file ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Benchmarks/fromStereoFile)
add_executable(benchmark_stereo_file Benchmarks/fromStereoFile/benchmark.cc)
target_link_libraries(benchmark_stereo_file ${PROJECT_NAME})

# KITTI Mono Example
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

# TUM RGB-D Example
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(RGBD-tum Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(RGBD-tum ${PROJECT_NAME})

# Camera Calibration Tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/camera_calibration_3D)
add_executable(Camera_Calibration_3D tools/camera_calibration_3D/camera_calibration_tool.cpp)
target_link_libraries(Camera_Calibration_3D ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/camera_calibration_monocular)
add_executable(Camera_Calibration_Mono tools/camera_calibration_monocular/camera_calibration.cpp)
target_link_libraries(Camera_Calibration_Mono ${PROJECT_NAME})

# Build test
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test)
add_executable(Test Test/Test.cc)
target_link_libraries(Test ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test)
add_executable(Test_RealSense Test/Test_RealSense.cc)
target_link_libraries(Test_RealSense ${PROJECT_NAME} ${realsense2_LIBRARY})

# Build V-SLAM application
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Drone)
add_executable(DroneVslam Drone/DroneVslam.cc)
target_link_libraries(DroneVslam ${PROJECT_NAME})

# Here we unset the CMAKE_BUILD_TYPE because -O3 option causes a crash
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/grab_image)
add_executable(Grab_Image tools/grab_image/grabImage.cc)
target_compile_options(Grab_Image PRIVATE -O0)
target_link_libraries(Grab_Image ${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/grab_image_realsense_rgb)
add_executable(Grab_Image_RealSense tools/grab_image_realsense_rgb/grabImageRealSense.cc)
target_link_libraries(Grab_Image_RealSense ${PROJECT_NAME} ${realsense2_LIBRARY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/grab_image_realsense_depth)
add_executable(Grab_Image_RealSense_Depth tools/grab_image_realsense_depth/grabImageRealSenseDepth.cc)
target_link_libraries(Grab_Image_RealSense_Depth ${PROJECT_NAME} ${realsense2_LIBRARY})