//Example code: A simple server side code, which echos back the received message.
//Handle multiple socket connections with select and fd_set on Linux
#include <stdio.h>
#include <string.h> //strlen
#include <stdlib.h>
#include <errno.h>
#include <unistd.h> //close
#include <arpa/inet.h> //close
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h> //FD_SET, FD_ISSET, FD_ZERO macros
#include <stdbool.h>
#include <signal.h>

#include "cobot_plan_control/matrix_dio.h"

#define TRUE 1
#define FALSE 0
#define PORT 8888
#define MAX_CLIENT 3

// char* get_timed_input(long timeout_seconds, char* input_buffer, int buffer_size);
int master_socket;
int client_socket[MAX_CLIENT];

void handle_sigint() {
  printf("handle_sigint()");
  for (int i = 0; i < MAX_CLIENT; i++) {
    int sd = client_socket[i];
    if( client_socket[i] != 0 )
      close(sd);
  }
  close(master_socket);
  // pid_t iPid = getpid(); /* Process gets its id.*/
  // kill(iPid, SIGINT);  /* Process sends itself a  SIGINT signal */
}

int main(int argc , char *argv[])
{
	int opt = TRUE;
	int addrlen , new_socket , activity , valread, i, sd;
	int max_sd;
	struct sockaddr_in address;

	char buffer[513]; //data buffer of 1K

	//set of socket descriptors
	fd_set readfds;

	//a message
	char *message = "ECHO Daemon v1.0 \r\n";

  // prepare struct
  sigset_t set;
  sigemptyset( &set );
  sigaddset( &set, SIGINT );

  struct sigaction sa;
  sa.sa_mask = set;
  sa.sa_handler = handle_sigint;
  sa.sa_flags = 0; // Restart functions if
                            // interrupted by handler

  sigaction( SIGINT, &sa, NULL );

/*
  signal(SIGHUP, handle_sigup);
  signal(SIGINT, handle_sigint);
  signal(SIGQUIT, handle_sigquit);
  signal(SIGILL, handle_sigill);
  signal(SIGTRAP, handle_sigtrap);
  signal(SIGABRT, handle_sigabrt);
  signal(SIGKILL, handle_sigkill);
*/

  //initialise all client_socket[] to 0 so not checked
	for (i = 0; i < MAX_CLIENT; i++)
	{
		client_socket[i] = 0;
	}

	//create a master socket
	if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//set master socket to allow multiple connections ,
	//this is just a good habit, it will work without this
	if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt,
		sizeof(opt)) < 0 )
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

  srand(time(NULL));   // Initialization, should only be called once.
  int portno;

	//type of socket created
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;

  LB_PORT_NUM:
  portno = (rand()%16383)+49152;
	address.sin_port = htons( portno );

	//bind the socket to localhost port 8888
	if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)
	{
    perror("bind failed");
    goto LB_PORT_NUM;
	}
  char file_name[] = "/home/mtec/catkin_ws/src/cobot/cobot_plan_control/cobot_server_setting.txt";
  FILE *fp = fopen(file_name, "w");
  fprintf(fp, "%d\n", portno);
  if(fp)
    fclose(fp);
	printf("Listener on port %d \n", portno);

	//try to specify maximum of 3 pending connections for the master socket
	if (listen(master_socket, 3) < 0)
	{
		perror("listen");
    goto LB_PORT_NUM;
	}

  short gpio = GPIO_Init();
  printf("GPIO_Init : %d\n", gpio);
  if(gpio != 0)
    return -1;
  GPO_Write(0);

	//accept the incoming connection
	addrlen = sizeof(address);
	puts("Waiting for connections ...");

  bool out = false;
	while(!out) {
		//clear the socket set
		FD_ZERO(&readfds);

		//add master socket to set
		FD_SET(master_socket, &readfds);
		max_sd = master_socket;

		//add child sockets to set
		for ( i = 0 ; i < MAX_CLIENT ; i++) {
			//socket descriptor
			sd = client_socket[i];

			//if valid socket descriptor then add to read list
			if(sd > 0)
				FD_SET( sd , &readfds);

			//highest file descriptor number, need it for the select function
			if(sd > max_sd)
				max_sd = sd;
		}

		//wait for an activity on one of the sockets , timeout is NULL ,
		//so wait indefinitely
		activity = select( max_sd + 1 , &readfds , NULL , NULL , NULL);

		if ((activity < 0) && (errno!=EINTR)) {
			printf("select error");
		}

		//If something happened on the master socket ,
		//then its an incoming connection
		if (FD_ISSET(master_socket, &readfds)) {
			if ((new_socket = accept(master_socket,
					(struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
				perror("accept");
				exit(EXIT_FAILURE);
			}

			//inform user of socket number - used in send and receive commands
			printf("New connection , socket fd is %d , ip is : %s , port : %d\n" , new_socket , inet_ntoa(address.sin_addr) , ntohs (address.sin_port));

			//send new connection greeting message
			if( send(new_socket, message, strlen(message), 0) != strlen(message) ) {
				perror("send");
			}

			puts("Welcome message sent successfully");

			//add new socket to array of sockets
			for (i = 0; i < MAX_CLIENT; i++) {
				//if position is empty
				if( client_socket[i] == 0 ) {
					client_socket[i] = new_socket;
					printf("Adding to list of sockets as %d\n" , i);

					break;
				}
			}
		}

		//else its some IO operation on some other socket
		for (i = 0; i < MAX_CLIENT; i++) {
			sd = client_socket[i];

			if (FD_ISSET( sd , &readfds)) {
				//Check if it was for closing , and also read the
				//incoming message
        bzero(buffer,513);
        printf("AAAAAAAAAAAAAAAAaa\n");
				if ((valread = read( sd , buffer, 512)) == 0) {
					//Somebody disconnected , get his details and print
					getpeername(sd , (struct sockaddr*)&address , \
						(socklen_t*)&addrlen);
					printf("Host disconnected , ip %s , port %d \n" ,
						inet_ntoa(address.sin_addr) , ntohs(address.sin_port));

					//Close the socket and mark as 0 in list for reuse
					close( sd );
					client_socket[i] = 0;
				}

				//Echo back the message that came in
				else {
					//set the string terminating NULL byte on the end
					//of the data read
					// buffer[valread] = '\0';
					// send(sd , buffer , strlen(buffer) , 0 );
          printf("Message : %s\n", buffer);
          if( buffer[0] == 'q')
            out = true;
          else if( buffer[0] == '+')
            printf("GPO_Write(%d) : %d\n", 255, GPO_Write(255));
          else if( buffer[0] == '-')
            printf("GPO_Write(%d) : %d\n", 0, GPO_Write(0));
				}
        printf("BBBBBBBBBBBBBBBB\n");
			}
		}
	}

  for (i = 0; i < MAX_CLIENT; i++) {
    sd = client_socket[i];
    if( client_socket[i] != 0 )
      close(sd);
  }
  close(master_socket);
	return 0;
}

/*
char* get_timed_input(long timeout_seconds, char* input_buffer, int buffer_size)
{
  time_t start, now;
  int nChars = 0;
  int chr = 0;
  const int VK_RETURN = 13;
  while( nChars < buffer_size && chr != VK_RETURN) {
    // start timer
    time(&start);
    while( !_kbhit() ) {
      time(&now);
      if( (now-start) >= timeout_seconds) {
        // timeout
        input_buffer[nChars] = 0;
        return NULL;
      }
    }
    chr = getche();
    input_buffer[nChars++] = chr;
  }
  // null terminate input buffer
  input_buffer[nChars] = 0;
  return input_buffer;
}
*/
