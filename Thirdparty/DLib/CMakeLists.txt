cmake_minimum_required(VERSION 3.0)
project(DLib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING")

set(HDRS_DLIB
  include/Random.h
  include/Timestamp.h)
set(SRCS_DLIB
  src/Random.cpp
  src/Timestamp.cpp)

find_package(OpenCV 4.2.0 QUIET)
  if(NOT OpenCV_FOUND)
     message(FATAL_ERROR "OpenCV > 4.2.0 not found.")
  endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DLib SHARED ${SRCS_DLIB})
target_link_libraries(DLib ${OpenCV_LIBS})

