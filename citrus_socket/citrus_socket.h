#ifndef CITRUS_SOCKET_CITRUS_SOCKET_H_
#define CITRUS_SOCKET_CITRUS_SOCKET_H_

#include <string>
#include <cstdint>
#include "destination.h"
#include "network_serialize.h"

class CitrusSocket {
 public:
  CitrusSocket(uint16_t port);
  ~CitrusSocket();
  uint16_t GetPort() { return port_; }

  void Send(const std::string& to_send, Destination to);
  void Send(const SerializedData& to_send, Destination to);
  SerializedData Receive();
  std::string ReceiveString();
  std::string Receive(Destination from);

  bool HasPackets();

 private:
  uint16_t port_;
  int socket_;
};

#endif /* CITRUS_SOCKET_CITRUS_SOCKET_H_ */
