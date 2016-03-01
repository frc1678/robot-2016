#include "data_sender.h"
#include "mutex"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include <iostream>

void sendData();

std::mutex position_mutex;
cv::Mat image_;
TrackerResults position_;

void vision::startSending() {
  cv::namedWindow("detected", cv::WINDOW_AUTOSIZE);
  while (true) {
    muan::sleep_for(0.1 * s);
    ::sendData();
  }
}

void sendData() {
  std::lock_guard<std::mutex> guard(position_mutex);
  if (image_.data) {
    cv::imshow("detected", image_);
    cv::waitKey(1);
  }
  // TODO(Lucas): send data

  if (position_.is_found) {
    std::cout << "angle to target: " << position_.angle.to(deg)
              << " degrees\nlag: "
              << (muan::now() - position_.time_captured).to(s) << "seconds\n"
              << std::endl;
  } else {
    std::cout << "not found\nlag: "
              << (muan::now() - position_.time_captured).to(s) << " seconds\n"
              << std::endl;
  }
}
void visionupdateData(cv::Mat image, TrackerResults position) {
  std::lock_guard<std::mutex> guard(position_mutex);
  position_ = position;
  image_ = image;
}
