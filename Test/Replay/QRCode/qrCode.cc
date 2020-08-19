#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <QrCodeTracker.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    cerr << endl << "Usage: ./qrCode_replay" << endl 
                 << "         path_to_image_file" << endl;
    return 1;
  }

  Mat inputImage;
  inputImage = imread(argv[1]);

  try {
    cout << endl << "-------" << endl;
    cout << "Start processing image file ..." << endl;
    QrCodeTracker *qrCodeTracker = new QrCodeTracker();
    qrCodeTracker->Check(inputImage);
    std::string decodedData;
    if (qrCodeTracker->getDecodedData(decodedData)) {
      cout << decodedData << endl;
    } else {
      cout << "No data detected" << endl;
    }
    cout << "Bounding Box: " << qrCodeTracker->getBoundingBox() << endl;
    cout << "Bounding Box Center: " << qrCodeTracker->getBoundingBoxCenter() << endl;
    // qrCodeTracker->setThresholds(98, 97);
    if (qrCodeTracker->isInsideBbox()) cout << "QR Code is inside bounding box" << endl; else cout << "QR Code is outside bounding box" << endl;

    if (qrCodeTracker->getDecodedData(decodedData) && qrCodeTracker->isInsideBbox()) {
      cout << "Recording data..." << endl;
      // Fake SLAM position
      cv::Point fakeSLAMPos(10,20); // this position must be taken from the SLAM current position...
      QrCode newQrCode(decodedData, qrCodeTracker->getBoundingBoxCenter(), fakeSLAMPos);
      qrCodeTracker->addQrCodeToMap(newQrCode);
      std::vector<QrCode> * qrCodeList = qrCodeTracker->getQrCodeList();
      std::cout << "From list " << qrCodeList->at(0).getCode() << std::endl;
    }

    qrCodeTracker->display();
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  for(;;)
  {
    int key = waitKey(10);
    if( key == 32) {
      cout << "Shutting down..." << endl;
      break;
    }
  }

  return 0;
}
