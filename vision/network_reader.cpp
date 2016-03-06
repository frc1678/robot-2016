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
#include <vector>

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
  char test[257];
  std::string str="";
  int i;
  for(i=0; str.find("\r\n\r\n")==std::string::npos; i++)
  {
      bzero(test, 257);
      n=read(sockfd, test, 256);
      if(n==0) break;
      str += test;
  }
  if(i==0) {
    close(sockfd);
    throw(std::string("ERROR no data"));
  }
  std::string str2="";
  for(i=0; str2.find("\r\n\r\n")==std::string::npos; i++)
  {
      bzero(test, 257);
      n=read(sockfd, test, 256);
      if(n==0) break;
      str2 += test;
  }
  str+=str2;
  if(i==0) {
    close(sockfd);
    throw(std::string("ERROR no data"));
  }
  close(sockfd);
  return str;
}
std::string vision::getResponseBody(std::string reply) {
  std::string delimiter = "\r\n\r\n";
  std::vector<std::string> tokens;
  size_t pos = 0;
  std::string token;
  while ((pos = reply.find(delimiter)) != std::string::npos) {
    token = reply.substr(0, pos);
    if(token != "") tokens.push_back(token);
    reply.erase(0, pos + delimiter.length());
  }
  std::string header=tokens[0];
  std::string body=tokens[1];
  tokens.clear();
  delimiter = "\r\n";
  pos = 0;
  while ((pos = header.find(delimiter)) != std::string::npos) {
    token = header.substr(0, pos);
    if(token != "") tokens.push_back(token);
    header.erase(0, pos + delimiter.length());
  }
  if(tokens[0] != "HTTP/1.1 200 OK" && tokens[0] != "HTTP/1.0 200 OK"){
    throw("ERROR could not get valid response: "+tokens[0]);
  }
  return body;
}
