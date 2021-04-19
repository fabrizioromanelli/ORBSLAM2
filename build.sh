#!/bin/bash
# Usage: ./build.sh <parallel_jobs> <build_type> <ros>
#        parallel_jobs: -jx where x is the number of threads
#        build_type   : could be one of the following: Release or Debug
#        jetson_build : could be: ON or OFF
#        cuda         : could be: ON or OFF
#        ros          : could be ROS or left blank for standard compilation

BUILD_TYPE=$2

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

echo "Configuring and building Thirdparty/UWB ..."
cd ../../uwb
sudo cp libUWBranging.so /usr/local/lib/

cd ../../

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

BUILDARGS="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

mkdir -p build
cd build
if [ "$BUILD_TYPE" == "Release" ] || [ "$BUILD_TYPE" == "Debug" ]; then
  if [ "$3" == "ON" ]; then
    BUILDARGS="${BUILDARGS} -DJETSON_BUILD=ON"
  else
    BUILDARGS="${BUILDARGS}"
  fi

  if [ "$4" == "ON" ]; then
    BUILDARGS="${BUILDARGS} -DCUDA_BUILD=ON"
  else
    BUILDARGS="${BUILDARGS}"
  fi

echo $BUILDARGS

  cmake .. $BUILDARGS
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
