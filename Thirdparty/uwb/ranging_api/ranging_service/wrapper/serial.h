#ifndef SERIAL_H
#define SERIAL_H
int serial_open(const char* portname);
int serial_set_interface_attribs(int fd, int speed);
void serial_set_mincount(int fd, int mcount);
#endif
