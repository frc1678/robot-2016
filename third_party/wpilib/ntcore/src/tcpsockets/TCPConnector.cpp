/*
   TCPConnector.h

   TCPConnector class definition. TCPConnector provides methods to actively
   establish TCP/IP connections with a server.

   ------------------------------------------

   Copyright � 2013 [Vic Hargrave - http://vichargrave.com]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License
*/

#include "TCPConnector.h"

#include <errno.h>
#include <fcntl.h>
#include <cstdio>
#include <cstring>
#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#else
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <unistd.h>
#endif

#include "TCPStream.h"

#include "llvm/SmallString.h"
#include "../Log.h"
#include "SocketError.h"

using namespace tcpsockets;

static int ResolveHostName(const char* hostname, struct in_addr* addr) {
  struct addrinfo hints;
  struct addrinfo* res;

  hints.ai_flags = 0;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = 0;
  hints.ai_addrlen = 0;
  hints.ai_addr = nullptr;
  hints.ai_canonname = nullptr;
  hints.ai_next = nullptr;
  int result = getaddrinfo(hostname, nullptr, &hints, &res);
  if (result == 0) {
    std::memcpy(addr, &((struct sockaddr_in*)res->ai_addr)->sin_addr,
                sizeof(struct in_addr));
    freeaddrinfo(res);
  }
  return result;
}

std::unique_ptr<NetworkStream> TCPConnector::connect(const char* server,
                                                     int port, int timeout) {
#ifdef _WIN32
  struct WSAHelper {
    WSAHelper() {
      WSAData wsaData;
      WORD wVersionRequested = MAKEWORD(2, 2);
      WSAStartup(wVersionRequested, &wsaData);
    }
    ~WSAHelper() { WSACleanup(); }
  };
  static WSAHelper helper;
#endif
  struct sockaddr_in address;

  std::memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  if (ResolveHostName(server, &(address.sin_addr)) != 0) {
#ifdef _WIN32
    llvm::SmallString<128> addr_copy(server);
    addr_copy.push_back('\0');
    int size = sizeof(address);
    if (WSAStringToAddress(addr_copy.data(), PF_INET, nullptr, (struct sockaddr*)&address, &size) != 0) {
      ERROR("could not resolve " << server << " address");
      return nullptr;
    }
#else
    inet_pton(PF_INET, server, &(address.sin_addr));
#endif
  }
  address.sin_port = htons(port);

  if (timeout == 0) {
    int sd = socket(AF_INET, SOCK_STREAM, 0);
    if (sd < 0) {
      ERROR("could not create socket");
      return nullptr;
    }
    if (::connect(sd, (struct sockaddr*)&address, sizeof(address)) != 0) {
      ERROR("connect() to " << server << " port " << port << " failed: " << SocketStrerror());
#ifdef _WIN32
      closesocket(sd);
#else
      ::close(sd);
#endif
      return nullptr;
    }
    return std::unique_ptr<NetworkStream>(new TCPStream(sd, &address));
  }

  fd_set sdset;
  struct timeval tv;
  socklen_t len;
  int result = -1, valopt, sd = socket(AF_INET, SOCK_STREAM, 0);
  if (sd < 0) {
    ERROR("could not create socket");
    return nullptr;
  }

  // Set socket to non-blocking
#ifdef _WIN32
  u_long mode = 1;
  ioctlsocket(sd, FIONBIO, &mode);
#else
  long arg;
  arg = fcntl(sd, F_GETFL, nullptr);
  arg |= O_NONBLOCK;
  fcntl(sd, F_SETFL, arg);
#endif

  // Connect with time limit
  if ((result = ::connect(sd, (struct sockaddr*)&address, sizeof(address))) <
      0) {
    int my_errno = SocketErrno();
#ifdef _WIN32
    if (my_errno == WSAEWOULDBLOCK || my_errno == WSAEINPROGRESS) {
#else
    if (my_errno == EWOULDBLOCK || my_errno == EINPROGRESS) {
#endif
      tv.tv_sec = timeout;
      tv.tv_usec = 0;
      FD_ZERO(&sdset);
      FD_SET(sd, &sdset);
      if (select(sd + 1, nullptr, &sdset, nullptr, &tv) > 0) {
        len = sizeof(int);
        getsockopt(sd, SOL_SOCKET, SO_ERROR, (char*)(&valopt), &len);
        if (valopt) {
          ERROR("select() to " << server << " port " << port << " error " << valopt << " - " << SocketStrerror(valopt));
        }
        // connection established
        else
          result = 0;
      } else
        INFO("connect() to " << server << " port " << port << " timed out");
    } else
      ERROR("connect() to " << server << " port " << port << " error " << SocketErrno() << " - " << SocketStrerror());
  }

  // Return socket to blocking mode
#ifdef _WIN32
  mode = 0;
  ioctlsocket(sd, FIONBIO, &mode);
#else
  arg = fcntl(sd, F_GETFL, nullptr);
  arg &= (~O_NONBLOCK);
  fcntl(sd, F_SETFL, arg);
#endif

  // Create stream object if connected, close if not.
  if (result == -1) {
#ifdef _WIN32
      closesocket(sd);
#else
      ::close(sd);
#endif
    return nullptr;
  }
  return std::unique_ptr<NetworkStream>(new TCPStream(sd, &address));
}
