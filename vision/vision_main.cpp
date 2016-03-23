#include "object_detector.h"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include "data_sender.h"
#include <thread>
#include "vision_utils.h"
#include <iostream>

int main() {
  cv::VideoCapture camera;

  // open last camera index
  int cameraIndex=0;
  for(int i=0; i<8; i++) {
    camera.open(i);
    if(camera.isOpened()) cameraIndex = i;
  }

  // exit if no camera can be opened
  camera.open(cameraIndex);
  if(!camera.isOpened()) return 0;

  ObjectTracker tracker = ObjectTracker();

  // for accurate lag measurements, run sender in another thread
  std::thread sender(vision::startSending);

  while (true) {
    cv::Mat image;

    // time is for lag measurement
    Time captureTime = muan::now();

    // exit if camera has lost connection
    // This does not work with normal OpenCV because of issue 5746.
    // Using https://github.com/amannababanana/opencv for fix.
    if(!camera.read(image)) return 0;

    TrackerResults position = tracker.Update(image);
    position.time_captured = captureTime;
    vision::updateData(image, position);
    image.release();
  }
  sender.join();
  return 0;
}
