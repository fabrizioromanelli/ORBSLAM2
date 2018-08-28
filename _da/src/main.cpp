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
    VideoCapture cap("/home/felix/Desktop/Kaefer1.mp4");

    if(!cap.isOpened())
        return -1;

    Mat t;
    for(int i = 0; i < 2500; i++)
    {
        cap >> t;
    }

    //voc file, settings file, sensor type, use viewer, save map
    ORB_SLAM2::System slam(path_vocab, path_yaml, ORB_SLAM2::System::MONOCULAR, true, true);

    Mat curr_frame;
    auto start_time = std::chrono::steady_clock::now();
    while(true)
    {
        Mat tmp;
        cap >> tmp;
        cap >> tmp;


        resize(tmp, curr_frame, slam_resolution);

        cvtColor(curr_frame, curr_frame, COLOR_BGR2GRAY);


        auto now = std::chrono::steady_clock::now();
        auto curr_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count();
        double curr_time = curr_time_ms / 1000.0f;


        imshow("werner", curr_frame);


        auto t1 = std::chrono::high_resolution_clock::now();
        slam.TrackMonocular(curr_frame, curr_time);
        auto t2 = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "SLAM Time: " << duration.count() << std::endl;

        //if(waitKey(1) >= 0)
//        {
//            if(!slam.GetKeyFrames().empty() && false)
//            {
//                std::cout << "X: " << slam.GetKeyFrames().back()->GetPose().at<float>(0, 3) << ", Y:";
//                std::cout << slam.GetKeyFrames().back()->GetPose().at<float>(2, 3) << std::endl << std::flush;
//
//            }
//        }



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