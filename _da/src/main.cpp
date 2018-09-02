//
// Created by felix on 20.08.18.
//


#include <iostream>
#include <chrono>
#include <unistd.h>

#include "System.h"

#include "opencv2/opencv.hpp"
#include "pangolin/pangolin.h"

using namespace cv;

const std::string path_vocab = "/home/felix/Desktop/ORB_SLAM2/Vocabulary/ORBvoc.bin";
const std::string path_yaml = "/home/felix/Desktop/ORB_SLAM2/_da/oneplus.yaml";
const std::string path_dataset = "/home/felix/Desktop/kitti/dataset/sequences/00";

const Size slam_resolution(720, 480);

int main(int, char**)
{
    //VideoCapture cap("http://10.0.0.12:8080/video");
    VideoCapture cap("http://tr3z0r.synology.me:8080/?action=stream");



    std::cout << "starting.." << std::endl;

    if(!cap.isOpened())
        return -1;


    //voc file, settings file, sensor type, use viewer, save map
    ORB_SLAM2::System slam(path_vocab, path_yaml, ORB_SLAM2::System::MONOCULAR, true, false);

    Mat curr_frame;

    auto start_time = std::chrono::steady_clock::now();
    while(true)
    {
        cap >> curr_frame;
        cvtColor(curr_frame, curr_frame, COLOR_BGR2GRAY);


        auto now = std::chrono::steady_clock::now();
        auto curr_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count();
        double curr_time = curr_time_ms / 1000.0f;


        slam.TrackMonocular(curr_frame, curr_time);

        std::cout << slam.GetCurrentCameraPose() << std::endl;

        if(waitKey(1) >= 0) break;


    }


    slam.Shutdown();

    cap.release();















/*
    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame;
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }
*/

    destroyAllWindows();
    return 0;
}