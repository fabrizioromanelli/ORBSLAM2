#!/bin/bash
mkdir -p Results
./fromMonocularFile/benchmark_mono_file ../Vocabulary/orb_mur.fbow ../Config/Mono-EuRoC.yaml ../../Datasets/EuRoC/MH_05/mav0/cam0/data ../Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/SLAM_monocular_euroc_MH05_file.dat
./fromMonocularFile/benchmark_mono_file ../Vocabulary/orb_mur.fbow ../Config/Mono-EuRoC.yaml ../../Datasets/EuRoC/MH_05/mav0/cam0/data ../Config/Mono-EuRoC_TimeStamps/MH05.txt > Results/LOC_monocular_euroc_MH05_file.dat
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/orb_mur.fbow ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg3_long_office_household ../Config/RGB-D-associations/fr3_long_office_household.txt > Results/SLAM_rgbd_tum_rgbd_dataset_freiburg3_long_office_household_file.dat
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/orb_mur.fbow ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg3_long_office_household ../Config/RGB-D-associations/fr3_long_office_household.txt > Results/LOC_rgbd_tum_rgbd_dataset_freiburg3_long_office_household_file.dat
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/orb_mur.fbow ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg2_pioneer_slam ../Config/RGB-D-associations/fr2_pioneer_slam.txt > Results/SLAM_rgbd_tum_rgbd_dataset_freiburg2_pioneer_slam_file.dat
./fromRGBDFile/benchmark_rgbd_file ../Vocabulary/orb_mur.fbow ../Config/RGB-D-TUM3.yaml ../../Datasets/TUM/rgbd_dataset_freiburg2_pioneer_slam ../Config/RGB-D-associations/fr2_pioneer_slam.txt > Results/LOC_rgbd_tum_rgbd_dataset_freiburg2_pioneer_slam_file.dat
./fromStereoFile/benchmark_stereo_file ../Vocabulary/orb_mur.fbow ../Config/Stereo-KITTI04-12.yaml ../../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/SLAM_stereo_kitti_15_file.dat
./fromStereoFile/benchmark_stereo_file ../Vocabulary/orb_mur.fbow ../Config/Stereo-KITTI04-12.yaml ../../Datasets/KITTI/data_odometry_gray/dataset/sequences/15 > Results/LOC_stereo_kitti_15_file.dat
mv *.map Results/
cd ..
