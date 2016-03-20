#include <iostream>
#include "citrus_socket/network_serialize.h"
#include "citrus_socket/citrus_socket.h"
int main() {
  auto connection=CitrusSocket(16782);
  bool found;
  while(true) {
    try {
      nlohmann::json json_recv = to_json(connection.Receive());
      std::cout<<"found: " << json_recv["found"] << "\nangle: " << json_recv["angle"]
          << "\nlag: " << json_recv["lag"] << std::endl;
    } catch (...) {}
  }
}
