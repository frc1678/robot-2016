#include "network_reader.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <vector>
#include <sstream>

std::string vision::readFromNetwork(std::string URL, int port, std::string filename) {
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    close(sockfd);
    throw(std::string("ERROR opening socket"));
  }
  struct hostent *server = gethostbyname(URL.c_str());
  if (server == NULL) {
    close(sockfd);
    throw(std::string("ERROR no such host"));
  }
  struct sockaddr_in serv_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
      (char *)&serv_addr.sin_addr.s_addr,
      server->h_length);
  serv_addr.sin_port = htons(port);

  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
    close(sockfd);
    throw(std::string("ERROR connecting"));
  }
  std::string request="GET "+filename+"  HTTP/1.1\r\nHost: "+URL+":"+std::to_string(port)+"\r\n\r\n";
  int n = write(sockfd,request.c_str(), request.length());
  if (n < 0) {
    close(sockfd);
    throw(std::string("ERROR writing to socket"));
  }
  char buffer[1024*1024+1];
  bzero(buffer, 1024*1024+1);
  n=read(sockfd, buffer, 1024*1024);
  if(n==0) {
    close(sockfd);
    throw(std::string("ERROR no data"));
  }
  close(sockfd);
  size_t pos = 0;
  std::string replyAsString(buffer);
  int content_length=0;
  int header_length=0;
  if((pos=replyAsString.find("\r\n\r\n"))!=std::string::npos) {
    std::string header(replyAsString.substr(0, pos));
    std::cout<<header;
    if(header.find("HTTP/1.1 200 OK\r\n")==std::string::npos&&header.find("HTTP/1.0 200 OK\r\n")){
      throw(std::string("ERROR could not get valid response or not found"));
    }
    header_length=header.length();
    while ((pos = header.find("\r\n\r\n")) != std::string::npos) {
      std::string token = header.substr(0, pos);
      if(token.find("Content-Length: ")!=std::string::npos) {
        token.erase(0, std::string("Content-Length: ").length());
        std::stringstream ss;
        ss<<token;
        ss>>content_length;
        if(ss.fail()) {
          throw(std::string("ERROR content length not an integer"));
        }
        if(content_length<1) {
          throw(std::string("ERROR content length < 1"));
        }
      }
      header.erase(0, pos + std::string("\r\n\r\n").length());
    }
  }
  else {
    throw(std::string("ERROR could not get valid response or status != 200"));
  }
  char* body=new char[content_length];
  memcpy(body, buffer+header_length+4, content_length);
  std::cout<<body<<std::endl;
  return "";
}
