#ifndef CITRUS_SOCKET_DESTINATION_
#define CITRUS_SOCKET_DESTINATION_

#include <string>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

class Destination {
 public:
  Destination(std::string address, uint32_t port);
  ~Destination();

  uint32_t GetPort();

  char* GetIpAddress();
  std::string GetIpAddressString();

  uint8_t GetIpSegmentCount();

  operator bool() const;

 private:
  uint32_t port_;
  hostent* hp_;
  //char[4] ipAddress;
  char* ipAddressString;
  struct addrinfo *res;

};

#endif /* CITRUS_SOCKET_DESTINATION_ */
