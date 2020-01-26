/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    cv::VideoCapture cap;
    // Open the camera on /dev/video2
    if(!cap.open(2))
    {
        cerr << endl << "Failed to open /dev/video2" << endl;
        return 0;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, false);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    // Main loop
    cv::Mat frame, resized;
    for(;;)
    {
        // Get current timestamp
        double currentTs = (double)cap.get(cv::CAP_PROP_POS_MSEC);
        // Get video stream current frame
        cap >> frame;
        resize(frame, resized, cv::Size(640, 480));

        if(resized.empty())
        {
            cerr << endl << "Failed to load frame!!!" << endl;
            return 1;
        }

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(resized, currentTs);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

//         double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//         // Wait to load the next frame
//         double T = 0;
//         if(ni  <nImages-1)
//             T = vTimestamps[ni+1] - tframe;
//         else if(ni>0)
//             T = tframe - vTimestamps[ni-1];

//         if(ttrack < T)
//             usleep((T - ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // // Tracking time statistics
    // sort(vTimesTrack.begin(), vTimesTrack.end());
    // float totaltime = 0;
    // for(int ni=0; ni<nImages; ni++)
    // {
    //     totaltime += vTimesTrack[ni];
    // }
    // cout << "-------" << endl << endl;
    // cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    // cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveTrajectory("CameraTrajectory.dat");
    // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");

    return 0;
}