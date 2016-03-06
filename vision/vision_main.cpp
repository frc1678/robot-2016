#include "object_detector.h"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include "data_sender.h"
#include <thread>
#include "vision_utils.h"

int main() {
  ObjectTracker tracker = ObjectTracker();
  //std::thread sender(vision::startSending);
  while (true) {
    Time captureTime = muan::now();
    cv::Mat image = vision::getImage("");
    TrackerResults position = tracker.Update(image);
    position.time_captured = captureTime;
    vision::updateData(image, position);
  }
  //sender.join();
  return 0;
}
