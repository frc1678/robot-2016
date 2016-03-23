#include "data_sender.h"
#include "citrus_socket/citrus_socket.h"
#include "citrus_socket/network_serialize.h"
#include "mutex"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include <iostream>

void sendData();

CitrusSocket connection(1678);
std::mutex position_mutex;
cv::Mat image_;
TrackerResults position_;

void vision::startSending() {
  cv::namedWindow("detected", cv::WINDOW_AUTOSIZE);
  while (true) {
    muan::sleep_for(.01 * s);
    ::sendData();
  }
}

void sendData() {
  position_mutex.lock();
  if (image_.data) {
    cv::imshow("detected", image_);
    cv::waitKey(1);
  }
  SerializedData data;
  if (position_.is_found) {
    nlohmann::json json_object = {{"found", true}, {"angle", position_.angle.to(deg)}, {"lag", (muan::now() - position_.time_captured).to(s)}};
    data << from_json(json_object);
    std::cout << "angle to target: " << position_.angle.to(deg)
              << " degrees\nlag: "
              << (muan::now() - position_.time_captured).to(s) << "seconds\n"
              << std::endl;
  } else {
    nlohmann::json json_object = {{"found", false}, {"angle", 0.0}, {"time", (muan::now() - position_.time_captured).to(s)}};
    data << from_json(json_object);
    std::cout << "not found\nlag: "
              << (muan::now() - position_.time_captured).to(s) << " seconds\n"
              << std::endl;
  }
  position_mutex.unlock();
  try {
    connection.Send(data, Destination("roborio-1678-frc.local", 16782));
  } catch (...) {
    std::cout<<"no connection"<<std::endl;
  }
}
void vision::updateData(cv::Mat image, TrackerResults position) {
  position_mutex.lock();
  position_ = position;
  image_ = image;
  position_mutex.unlock();
}
