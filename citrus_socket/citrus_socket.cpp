#include "citrus_socket.h"
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <poll.h>
#include <iostream>
#include <exception>
#include <string>
#include <cstring>

CitrusSocket::CitrusSocket(uint16_t port) {
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    throw "Cannot create the socket";
  }

  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);
  auto bind_result = bind(socket_, (sockaddr *)&address, sizeof(address));
  if (bind_result < 0) {
    std::cout<<"bind failed"<<std::endl;
    throw std::string("Bind failed");
  }

  uint32_t alen = sizeof(address);
  if (getsockname(socket_, (sockaddr *)&address, &alen) < 0) {
    std::cout<<"cannot getsockname"<<std::endl;
    throw std::string("Cannot getsockname");
  }
  port_ = ntohs(address.sin_port);
}

CitrusSocket::~CitrusSocket() { close(socket_); }

void CitrusSocket::Send(const std::string &to_send, Destination to) {
  sockaddr_in destaddr = {};
  destaddr.sin_family = AF_INET;
  destaddr.sin_port = htons(to.GetPort());
  memcpy((void *)&destaddr.sin_addr, to.GetIpAddress(), to.GetIpSegmentCount());
  auto result = sendto(socket_, to_send.c_str(), to_send.length(), 0,
                       (sockaddr *)&destaddr, sizeof(destaddr));
  if (result == -1) {
    // TODO(Kyle) Handle all possible errors
  }
}

void CitrusSocket::Send(const SerializedData &to_send, Destination to) {
  sockaddr_in destaddr = {};
  destaddr.sin_family = AF_INET;
  destaddr.sin_port = htons(to.GetPort());
  memcpy((void *)&destaddr.sin_addr, to.GetIpAddress(), to.GetIpSegmentCount());
  auto result = sendto(socket_, to_send.GetData(), to_send.GetLength(), 0,
                       (sockaddr *)&destaddr, sizeof(destaddr));
  if (result == -1) {
    // TODO(Kyle) Handle all possible errors
  }
}

std::string CitrusSocket::ReceiveString() {
  char buf[1024];  // TODO(Kyle) This should by dynamically allocated by
                   // checking the size with MSG_PEEK
  size_t recvlen = 0;

  pollfd poll_request;
  poll_request.fd = socket_;
  poll_request.events = 0xFFFF;
  auto poll_result = poll(&poll_request, 1, 10);
  if (poll_result == -1) {
    throw std::string("Polling failed");
  }
  // TODO(Kyle) Figure out exactly which events 0x40 corresponds to - it seems
  // to be just the bits which are high when there is a udp packet waiting
  if ((poll_request.revents & 0x40) > 0) {
    recvlen = recv(socket_, &buf[0], 1024, MSG_WAITALL);
  }
  return std::string(&buf[0], recvlen);
}

SerializedData CitrusSocket::Receive() {
  SerializedData ret_data;
  ret_data << ReceiveString();
  return ret_data;
}

bool CitrusSocket::HasPackets() {
  pollfd poll_request;
  poll_request.fd = socket_;
  poll_request.events = 0xFFFF;
  auto poll_result = poll(&poll_request, 1, 10);
  if (poll_result == -1) {
    throw std::string("Polling failed");
  }
  return (poll_request.revents & 0x40) > 0;
}
