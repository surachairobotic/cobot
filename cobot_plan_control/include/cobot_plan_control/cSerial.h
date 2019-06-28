#ifndef __C_SERIAL__
#define __C_SERIAL__

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions


class cSerial{
private:
  int fd, baud_rate;

public:
  char port_name[128];

  cSerial():baud_rate(0),fd(-1){
    port_name[0] = 0;
  }
  ~cSerial(){
    if( fd>=0 )
      close(fd);
  }

  bool open(const char *_port_name, int _baud_rate);
  int read(void *buf, unsigned len);
  void write(const char *buf);
};

#endif
