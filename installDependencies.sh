#!/bin/bash

# Packages needed by Pangolin
sudo apt-get install -y --no-install-recommends \
  cmake \
  ffmpeg \
  libavcodec-dev \
  libavdevice-dev \
  libavformat-dev \
  libavutil-dev \
  libegl1-mesa-dev \
  libgl1-mesa \
  libglew-dev \
  libpython2.7-dev \
  libswscale-dev \
  libwayland-dev \
  libxkbcommon-dev \
  pkg-config \
  wayland-protocols

cd ~/workspace || return 1

# Pangolin installation
git clone --single-branch --branch 'v0.6' --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin || return 1
mkdir build
cd build || return 1
cmake ..
cmake --build .
sudo make install "-j$(nproc --all)"

cd ~/workspace || return 1

# Eigen installation
git clone --single-branch --branch '3.4.0' --depth 1 https://gitlab.com/libeigen/eigen.git
cd eigen || return 1
mkdir build
cd build || return 1
cmake ..
sudo make install "-j$(nproc --all)"

cd ~/workspace || return 1

# This adds libjasper-dev old repo
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  libjasper1 \
  libjasper-dev

# Packages needed by OpenCV
sudo apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  libavcodec-dev \
  libavformat-dev \
  libdc1394-22-dev
  libgtk2.0-dev \
  libjpeg-dev \
  libpng-dev \
  libswscale-dev \
  libtbb2 \
  libtbb-dev \
  libtiff-dev \
  pkg-config \
  python-dev \
  python-numpy

# OpenCV installation
git clone --single-branch --branch '4.5.5' --depth 1 https://github.com/opencv/opencv.git
cd opencv || return 1
mkdir build
cd build || return 1
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install "-j$(nproc --all)"

# Boost installation
sudo apt-get install -y --no-install-recommends libboost-all-dev
