#include "cobot_plan_control/tcp_client.h"
#include <string>
#include <cstring>

bool tcpClient::init(int& _port) {
  portno = _port;
  printf("tcpClient::tcpClient() : portno = %d\n", portno);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    printf("tcpClient::tcpClient() : ERROR opening socket\n");
  server = gethostbyname("127.0.0.1");
  if (server == NULL) {
    printf("tcpClient::tcpClient() : ERROR, no such host\n");
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    printf("tcpClient::tcpClient() : ERROR connecting\n");
}

std::string tcpClient::tcpRead() {
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0)
    printf("tcpClient::tcpRead() : ERROR reading from socket\n");
  printf("%s\n",buffer);
  std::string str(buffer);
  return str;
}

bool tcpClient::tcpWrite(std::string& str) {
  char buff[str.size() + 1];
  strcpy(buff, str.c_str());
  n = write(sockfd,buff,strlen(buff));
  if (n < 0) {
    return false;
  }
  return true;
}
