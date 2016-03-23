#include "object_detector.h"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include "data_sender.h"
#include <thread>
#include "vision_utils.h"
#include <iostream>

int main() {
  cv::VideoCapture camera;
  int cameraIndex=0;
  for(int i=0; i<8; i++) {
    camera.open(i);
    if(camera.isOpened()) cameraIndex = i;
  }
  camera.open(cameraIndex);
  if(!camera.isOpened()) return 0;

  ObjectTracker tracker = ObjectTracker();
  std::thread sender(vision::startSending);

  while (true) {
    cv::Mat image;
    Time captureTime = muan::now();
    if(!camera.read(image)) return 0;
    TrackerResults position = tracker.Update(image);
    position.time_captured = captureTime;
    vision::updateData(image, position);
    image.release();
  }
  sender.join();
  return 0;
}
