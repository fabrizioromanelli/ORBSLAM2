#!/bin/bash
mkdir Results
./Benchmarks/fromMonocularFile/benchmark_mono_file Vocabulary/ORBvoc.bin Config/Mono-EuRoC.yaml ../Datasets/EuRoC/MH_05/mav0/cam0/data Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/SLAM_monocular_euroc_MH05_file.dat
./Benchmarks/fromMonocularFile/benchmark_mono_file Vocabulary/ORBvoc.bin Config/Mono-EuRoC.yaml ../Datasets/EuRoC/MH_05/mav0/cam0/data Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/LOC_monocular_euroc_MH05_file.dat
./Benchmarks/fromRGBDFile/benchmark_rgbd_file Vocabulary/ORBvoc.bin Config/RGB-D-TUM2.yaml ../Datasets/TUM/rgbd_dataset_freiburg2_pioneer_slam Config/RGB-D-associations/fr2_pioneer_slam.txt > Results/SLAM_rgbd_tum_rgbd_dataset_freiburg2_pioneer_slam_file.dat
./Benchmarks/fromRGBDFile/benchmark_rgbd_file Vocabulary/ORBvoc.bin Config/RGB-D-TUM2.yaml ../Datasets/TUM/rgbd_dataset_freiburg2_pioneer_slam Config/RGB-D-associations/fr2_pioneer_slam.txt > Results/LOC_rgbd_tum_rgbd_dataset_freiburg2_pioneer_slam_file.dat
./Benchmarks/fromStereoFile/benchmark_stereo_file Vocabulary/ORBvoc.bin Config/Stereo-KITTI04-12.yaml ../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/SLAM_stereo_kitti_15_file.dat
./Benchmarks/fromStereoFile/benchmark_stereo_file Vocabulary/ORBvoc.bin Config/Stereo-KITTI04-12.yaml ../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/LOC_stereo_kitti_15_file.dat
cd ..