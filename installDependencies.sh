#!/bin/bash

# Packages needed by Pangolin
sudo apt-get install -y --no-install-recommends \
  ffmpeg \
  libavcodec-dev \
  libavdevice-dev \
  libavformat-dev \
  libavutil-dev \
  libegl1-mesa-dev \
  libglew-dev \
  libpython2.7-dev \
  libswscale-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols

  # cmake \
  # pkg-config \
  # libgl1-mesa \

# Package for Ubuntu 22.04 WSL2
#sudo apt-get install libboost-all-dev libglew-dev libavdevice-dev

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

# Useless as Ubuntu 22.04 already has it.
# # Eigen installation
# git clone --single-branch --branch '3.4.0' --depth 1 https://gitlab.com/libeigen/eigen.git
# cd eigen || return 1
# mkdir build
# cd build || return 1
# cmake ..
# sudo make install "-j$(nproc --all)"

cd ~/workspace || return 1

# This adds libjasper-dev old repo
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
echo "If the previous fails, please add the key with apt-key:"
echo "sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 40976EAF437D05B5"
echo "Please check that the key is correct!"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  libjasper1 \
  libjasper-dev

# Packages needed by OpenCV
sudo apt-get install -y --no-install-recommends \
  libavcodec-dev \
  libavformat-dev \
  libgtk2.0-dev \
  libjpeg-dev \
  libpng-dev \
  libswscale-dev \
  libtbb2 \
  libtbb-dev \
  libtiff-dev

  # build-essential \
  # cmake \
  # git \
  # libdc1394-22-dev \
  # pkg-config \
  # python-dev \
  # python-numpy

# OpenCV installation
git clone --single-branch --branch '4.5.5' --depth 1 https://github.com/opencv/opencv.git
cd opencv || return 1
mkdir build
cd build || return 1
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install "-j$(nproc --all)"

# Boost installation
# sudo apt-get install -y --no-install-recommends libboost-all-dev
