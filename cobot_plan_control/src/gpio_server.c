/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>                // for gettimeofday()

#include <stdbool.h>

#include "cobot_plan_control/matrix_dio.h"

unsigned short dio = 256;

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[])
{
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) 
    error("ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = 2222;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");
  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
  if (newsockfd < 0)
    error("ERROR on accept");

  short gpio = GPIO_Init();
  printf("GPIO_Init : %d", gpio);
  if(gpio != 0)
    return -1;

//  printf("InitWDT : %d", InitWDT());
//  printf("StopWDT : %d", StopWDT());
  unsigned short *tmp;
  bool out = false;
  while(!out) {
    bzero(buffer,256);
    n = read(newsockfd,buffer,255);
    printf("Here is the message: %s\n",buffer);
    if (n < 0)
      error("ERROR reading from socket");
    else if( buffer[0] == 'q')
      out = true;
    else if( buffer[0] == '+')
      printf("GPO_Write(%d) : ", 0, GPO_Write(0));
    else if( buffer[0] == '-')
      printf("GPO_Write(%d) : ", 255, GPO_Write(255));

    if(!out) {
      GPO_Read(tmp);
      printf("GPO_Read : %d\n", *tmp);

      n = write(newsockfd,"I got your message",18);
      if (n < 0)
        error("ERROR writing to socket");
    }
  }
  close(newsockfd);
  close(sockfd);
  printf("end");
  return 0; 
}
