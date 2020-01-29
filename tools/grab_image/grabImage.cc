#include <string>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./Grab_Image video_device_id" << std::endl;
        return 1;
    }

    VideoCapture cap;

    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(atoi(argv[1]))){
      std::cerr << std::endl << "Cannot open /dev/video" << argv[1] << std::endl;
      return 0;
    }

    // std::cout << cap.get(CAP_PROP_FPS) << std::endl;

    // long previousTs = 0;

    for(;;)
    {
      Mat frame, resized;
      long currentTs = cap.get(CAP_PROP_POS_MSEC);
      // std::cout << currentTs - previousTs << std::endl;
      cap >> frame;
      resize(frame, resized, Size(640, 480));
      if( resized.empty() ) break; // end of video stream
      imshow("Grab images", resized);
      char filename_[] = "calib_";
      char *filename = &filename_[0];
      strcat(filename, std::to_string(currentTs).c_str());
      strcat(filename, ".jpg");

      int key = waitKey(10);
      // Save image to jpeg if Spacebar has been pressed
      if( key == 32 ) {
        imwrite(filename, resized);
      } else if (key == 27) { // stop capturing by pressing ESC
        break;
      }

      // previousTs = currentTs;
    }

    return 0;
}