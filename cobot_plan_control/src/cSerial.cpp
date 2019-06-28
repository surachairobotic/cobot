#include "cobot_plan_control/cSerial.h"

bool cSerial::open(const char *_port_name, int _baud_rate){
  int ret = 0;

  if( strlen(_port_name)>=128 ){
    printf("cSerial::open() : too long port name : %s\n", _port_name);
    return false;
  }

  strcpy( port_name, _port_name );
  baud_rate = _baud_rate;

  fd = ::open( port_name, O_RDWR| O_NONBLOCK );
  if ( fd < 0 ){
    printf("cSerial::open() : Cannot connect to port : %s\n", port_name);
    return false;
  }
  struct termios toptions;
  memset (&toptions, 0, sizeof(toptions));
  /* get current serial port settings */
  if ( tcgetattr ( fd, &toptions ) != 0 ){
    printf("cSerial::open() : tcgetattr() failed : %d\n", ret);
    return false;
  }
  tcgetattr(fd, &toptions);
  /* set 9600 baud both ways */
  cfsetispeed(&toptions, baud_rate);
  cfsetospeed(&toptions, baud_rate);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;   // No parity bit
  toptions.c_cflag &= ~CSTOPB;   // 1 stop bit
  toptions.c_cflag &= ~CSIZE;    // Mask data size
  toptions.c_cflag |= CS8;       // Select 8 data bits

  toptions.c_cflag     &=  ~CRTSCTS;           // no flow control
//  tty.c_cc[VMIN]   =  1;                  // read doesn't block
//  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
//  tty.c_cc[VMIN]   =  0;                  // polling read
//  tty.c_cc[VTIME]  =  0;
//  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
  /* Canonical mode */
  toptions.c_lflag |= ICANON;

  // flush port
  tcflush( fd, TCIFLUSH );
  /* commit the serial port settings */
  if( (ret=tcsetattr(fd, TCSANOW, &toptions))!=0 ){
    printf("cSerial::open() : tcsetattr() failed : %d\n", ret);
    return false;
  }
  return true;
}

int cSerial::read(void *buf, unsigned len){
  return ::read( fd, buf, len );
}

void cSerial::write(const char *buf){
  ::write( fd, (void*)buf, strlen(buf) );
}
