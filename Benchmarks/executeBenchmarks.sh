#!/bin/bash
mkdir Results
./fromMonocularFile/benchmark_mono_file ../Vocabulary/ORBvoc.bin ../Config/Mono-EuRoC.yaml ../../Datasets/EuRoC/MH_05/mav0/cam0/data ../Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/SLAM_monocular_euroc_MH05_file.dat
cp *.map Results/
./fromMonocularFile/benchmark_mono_file ../Vocabulary/ORBvoc.bin ../Config/Mono-EuRoC.yaml ../../Datasets/EuRoC/MH_05/mav0/cam0/data ../Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/LOC_monocular_euroc_MH05_file.dat
rm -rf *.map
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/ORBvoc.bin ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg3_long_office_household ../Config/RGB-D-associations/fr3_long_office_household.txt > Results/SLAM_rgbd_tum_rgbd_dataset_freiburg3_long_office_household_file.dat
cp *.map Results/
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/ORBvoc.bin ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg3_long_office_household ../Config/RGB-D-associations/fr3_long_office_household.txt > Results/LOC_rgbd_tum_rgbd_dataset_freiburg3_long_office_household_file.dat
rm -rf *.map
./fromStereoFile/benchmark_stereo_file ../Vocabulary/ORBvoc.bin ../Config/Stereo-KITTI04-12.yaml ../../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/SLAM_stereo_kitti_15_file.dat
cp *.map Results/
./fromStereoFile/benchmark_stereo_file ../Vocabulary/ORBvoc.bin ../Config/Stereo-KITTI04-12.yaml ../../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/LOC_stereo_kitti_15_file.dat
rm -rf *.map
cd ..