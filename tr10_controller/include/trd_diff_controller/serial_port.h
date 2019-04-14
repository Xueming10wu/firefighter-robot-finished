#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

int openSerialPort(void **handle, const char *port_name);
int closeSerialPort(void *handle);
int setupSerialPort(void *handle);

int writeData(void *handle, const char *buffer, int length);
int readData(void *handle, char *buffer, int length);

#endif
