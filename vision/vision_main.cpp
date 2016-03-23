#include "object_detector.h"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include "data_sender.h"
#include <thread>
#include "vision_utils.h"
#include <iostream>

int main(int numArgs, char **args) {
  //if(numArgs < 2) {
  //   std::cout << "provide camera number as argument" << std::endl
  //           << "suggested command: ls /dev | grep video | vision" <<std::endl;
  //   return -1;
  //}
  //int cameraIndex = args[numArgs - 1][5] - '0';

  ObjectTracker tracker = ObjectTracker();
  cv::VideoCapture camera;
  camera.open(1);
  if(!camera.isOpened()) camera.open(2);
  if(!camera.isOpened()) return 0;

  std::thread sender(vision::startSending);
  while (true) {
    cv::Mat image;
    Time captureTime = muan::now();
    if(!camera.read(image)) return 0;
    TrackerResults position  = tracker.Update(image);
    position.time_captured = captureTime;
    vision::updateData(image, position);
    image.release();
  }
  sender.join();
  return 0;
}
