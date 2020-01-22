# Usage: ./build.sh <parallel_jobs> <build_type> <ros>
#        parallel_jobs: -jx where x is the number of threads
#        build_type   : could be one of the following: Release or Debug
#        ros          : could be ROS or left blank for standard compilation

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $1

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
if [ "$2" == "Release" ] || [ "$2" == "Debug" ]; then
  cmake .. -DCMAKE_BUILD_TYPE=$2
else
  echo "[ERROR] Invalid build type. Should be one of the following: Release/Debug."
  exit 1
fi
make $1

cd ..

cd Vocabulary
echo "Converting vocabulary to binary version"
./bin_vocabulary
cd ..

if [ "$3" == "ROS" ]; then
  echo "Building ROS nodes"

  cd Examples/ROS/ORB_SLAM2
  mkdir -p build
  cd build
  cmake .. -DROS_BUILD_TYPE=Release
  make $1
fi