#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int serial_open(const char* portname) {
  return open(portname, O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY);
}

int serial_set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if(fcntl(fd, F_SETFL, 0) < 0) {
      perror("could not set fcntl");
      return 0;
    }

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return 0;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    /* Enable the receiver and set local mode */
    tty.c_cflag |= (CLOCAL | CREAD);
    /* Mask the character size bits and turn off (odd) parity */
    tty.c_cflag &= ~(CSIZE | PARENB | PARODD);
    /* Select 8 data bits */
    tty.c_cflag |= CS8;

    /* Raw input */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
        | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* Raw output */
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;   // read() blocks until it sees at least 1 byte
    tty.c_cc[VTIME] = 0;  // read() returns immediately after it sees data 

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return 0;
    }
    return 1;
}
