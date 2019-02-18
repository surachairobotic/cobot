#ifndef __TCP_CLIENT_H__
#define __TCP_CLIENT_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <sstream>

class tcpClient{

private:
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];


public:
  bool init();
  std::string tcpRead();
  bool tcpWrite(std::string& str);
};

#endif // __TCP_CLIENT_H__
