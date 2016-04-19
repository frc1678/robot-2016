#include "data_sender.h"
#include "citrus_socket/citrus_socket.h"
#include "citrus_socket/network_serialize.h"
#include "mutex"
#include "opencv2/highgui.hpp"
#include "muan/utils/timing_utils.h"
#include <iostream>

void sendData();

CitrusSocket connection(9999);
std::mutex position_mutex;
TrackerResults position_;

void vision::startSending() {
  while (true) {
    muan::sleep_for(.01 * s);
    ::sendData();
  }
}

void sendData() {
  // Set the contents of what is being sent over
  position_mutex.lock();
  SerializedData data;
  if (position_.is_found) {
    nlohmann::json json_object = {{"found", true}, {"angle", position_.angle.to(deg)}, {"lag", (muan::now() - position_.time_captured).to(s)}};
    data << from_json(json_object);
    std::cout << "angle to target: " << position_.angle.to(deg)
              << " degrees\nlag: "
              << (muan::now() - position_.time_captured).to(s) << "seconds\n"
              << std::endl;
  } else {
    nlohmann::json json_object = {{"found", false}, {"angle", 0.0}, {"lag", (muan::now() - position_.time_captured).to(s)}};
    data << from_json(json_object);
    std::cout << "not found\nlag: "
              << (muan::now() - position_.time_captured).to(s) << " seconds\n"
              << std::endl;
  }
  position_mutex.unlock();
  // Send the data
  try {
    connection.Send(data, Destination("citrus-vision.local", 9999));
  } catch (...) {
    std::cout<<"no connection"<<std::endl;
  }
}
void vision::updateData(TrackerResults position) {
  position_mutex.lock();
  position_ = position;
  position_mutex.unlock();
}
