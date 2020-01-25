#!/bin/bash

# Packages needed by Pangolin
sudo apt -y install libgl1-mesa
sudo apt -y install libglew-dev
sudo apt -y install cmake
sudo apt -y install libpython2.7-dev
sudo apt -y install pkg-config
sudo apt -y install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo apt -y install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev

cd ~/workspace
# Pangolin installation
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
sudo make install -j8

cd ~/workspace
# Eigen installation
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
cmake ..
sudo make install -j8

# This add libjasper-dev old repo
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev

# Packages needed by OpenCV
sudo apt-get -y install build-essential
sudo apt-get -y install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get -y install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

cd ~/workspace
# OpenCV installation
git clone git@github.com:opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install -j8

# Boost installation
sudo apt-get install libboost-all-dev