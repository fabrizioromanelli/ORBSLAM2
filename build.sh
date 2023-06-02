#!/bin/bash
# Usage: ./build.sh <parallel_jobs> <build_type> <jetson_switch> <realsense_switch> <ros>
#        parallel_jobs: -jx where x is the number of threads
#        build_type   : could be one of the following: Release or Debug
#        jetson_build : could be: ON or OFF
#        realsense    : could be: ON or OFF
#        ros          : could be ROS or left blank for standard compilation

BUILD_TYPE=$2
REALSENSE=$4

RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e " "
echo -e "${RED}If you are running Ubuntu 22.04 and libboost 1.74, please change file /usr/include/boost/serialization/list.hpp${NC}"
echo -e "${RED}adding the following: ${NC}"
echo -e "${RED}#include <boost/archive/detail/basic_iarchive.hpp>${NC}"
echo -e "${RED}#include <boost/serialization/version.hpp>${NC}"
echo -e "${RED}and substituting boost::serialization::library_version_type with:${NC}"
echo -e "${RED}boost::archive::library_version_type${NC}"
echo -e "${RED}This is a known bug in serialization boost module for version 1.74.${NC}"
echo -e " "

if [ "$1" == "" ]; then
  echo "No argument set for parallel jobs! Set -jx where x is the number of threads!"
  exit
elif [ "$BUILD_TYPE" == "" ]; then
  echo "No argument set for build type! Set Release or Debug. Now compiling in Release mode"
  BUILD_TYPE="Release"
fi

echo "Configuring and building Thirdparty/fbow ..."
cd Thirdparty/fbow
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=../ -DUSE_AVX=OFF
make install $1

echo "Configuring and building Thirdparty/DLib ..."
cd ../../DLib
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
make $1

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
make $1

cd ../../..

cd Vocabulary
VOCABULARYFILE=`pwd`"/orb_mur.fbow"
if test -f "$VOCABULARYFILE"; then
  echo "Vocabulary file already extracted."
else
  echo "Uncompress vocabulary ..."
  tar -xf orb_mur.fbow.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
if [ "$BUILD_TYPE" == "Release" ] || [ "$BUILD_TYPE" == "Debug" ]; then
  if [ "$3" == "ON" ]; then
    if [ $REALSENSE == "ON" ]; then
      cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DJETSON_BUILD=ON
    else
      cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DJETSON_BUILD=ON -DREALSENSE_BUILD=OFF
    fi
  else
    if [ $REALSENSE == "ON" ]; then
      cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    else
      cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DREALSENSE_BUILD=OFF
    fi
  fi
else
  echo "[ERROR] Invalid build type. Should be one of the following: Release/Debug."
  exit 1
fi
make $1

sudo make install
sudo ldconfig

cd ..

if [ "$5" == "ROS" ]; then
  echo "Building ROS nodes"

  cd Examples/ROS/ORB_SLAM2
  mkdir -p build
  cd build
  cmake .. -DROS_BUILD_TYPE=$BUILD_TYPE
  make $1
fi
