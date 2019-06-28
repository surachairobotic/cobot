/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <sys/time.h>                // for gettimeofday()
#include <stdbool.h>
#include <termios.h>

#include "cobot_plan_control/matrix_dio.h"

static struct termios old, new;
unsigned short dio = 256;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      new.c_lflag |= ECHO; /* set echo mode */
  } else {
      new.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void)
{
  return getch_(0);
}

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[])
{
  srand(time(NULL));   // Initialization, should only be called once.
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  LB_PORT_NUM:
  portno = (rand()%16383)+49152;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    error("ERROR on binding");
    goto LB_PORT_NUM;
  }
  char file_name[] = "/home/mtec/catkin_ws/src/cobot/cobot_plan_control/cobot_server_setting.txt";
  FILE *fp = fopen(file_name, "w");
  fprintf(fp, "%d\n", portno);
  if(fp)
    fclose(fp);

  if ( listen(sockfd, 3) < 0 ) {
    error("ERROR on listen");
    goto LB_PORT_NUM;
  }

  //accept the incoming connection
  clilen = sizeof(cli_addr);

  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
  if (newsockfd < 0)
    error("ERROR on accept");

  short gpio = GPIO_Init();
  printf("GPIO_Init : %d\n", gpio);
  if(gpio != 0)
    return -1;

  GPO_Write(0);

  unsigned short *tmp;
  while(1) {
    GPO_Read(tmp);
    printf("GPO_Read : %d\n", *tmp);
    GPI_Read(tmp);
    printf("GPI_Read : %d\n", *tmp);
    usleep(100000);
  }
  exit(1);


//  printf("InitWDT : %d", InitWDT());
//  printf("StopWDT : %d", StopWDT());
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
      printf("GPO_Write(%d) : %d\n", 255, GPO_Write(255));
    else if( buffer[0] == '-')
      printf("GPO_Write(%d) : %d\n", 0, GPO_Write(0));
    printf("GPO_Write is OK\n");

    if(!out) {
      /*
      GPO_Read(tmp);
      printf("GPO_Read : %d\n", *tmp);
      GPI_Read(tmp);
      printf("GPI_Read : %d\n", *tmp);
      */
      n = write(newsockfd,"I got your message",18);
      if (n < 0)
        error("ERROR writing to socket");
    }
    printf("GPO_Read is OK\n");
  }
  close(newsockfd);
  close(sockfd);
  printf("end");
  return 0;
}
