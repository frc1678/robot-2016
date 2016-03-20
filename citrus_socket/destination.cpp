#include "destination.h"
#include <sstream>

Destination::Destination(std::string address, uint32_t port) : port_(port) {
  hp_ = gethostbyname(address.c_str());
}

Destination::~Destination() {}

uint32_t Destination::GetPort() { return port_; }

char* Destination::GetIpAddress() { return hp_->h_addr_list[0]; }

uint8_t Destination::GetIpSegmentCount() { return hp_->h_length; }

Destination::operator bool() const {
  return hp_->h_length != 0;
}

std::string Destination::GetIpAddressString() {
  std::stringstream ss;
  auto addr = (unsigned char*)hp_->h_addr_list[0];
  ss << static_cast<int>(addr[0]) << "." << static_cast<int>(addr[1]) << "."
     << static_cast<int>(addr[2]) << "." << static_cast<int>(addr[3]);
  return ss.str();
}
