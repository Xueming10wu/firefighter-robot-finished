#include "trd_diff_controller/serial_port.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <assert.h>

int openSerialPort(void **handle, const char *port_name){
    int fd;
    /*
     * O_RDWR : read+write mode
     * O_NOCTTY : not a "controlling terminal" for this port
     * O_NDELAY : do not care what state the DCD signal line is in - whether the other end of the port is up and running
     */
    fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){
        fprintf(stderr, "Unable to open %s\n", port_name);
        return -3;
    }
    if(!isatty(fd)){
        fprintf(stderr, "%s is not a serial port\n", port_name);
        return -3;
    }
    *handle = (int *) malloc(sizeof(int));
    **(int **) handle = fd;
    return fd;
}

int closeSerialPort(void *handle){
    if(NULL == handle){
        return 0;
    }
    close(*(int *) handle);
    free(handle);
    return 0;
}

int setupSerialPort(void *handle){
    struct termios options;
    // get the current options for the port
    tcgetattr(*(int *) handle, &options);

    // 8 bits, no parity, 1 stop
    options.c_cflag  = 0;
    options.c_cflag |= CS8; // 8 bits input
    // enable the reveiver and set local mode
    options.c_cflag  |= (CLOCAL | CREAD);
    // set baudrates
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
    // no input processing
    options.c_iflag = 0;
    // no output processing
    options.c_oflag = 0;
    // no line processing
    options.c_lflag = 0;
    // read timeout
    options.c_cc[VMIN] = 0; // non-blocking (minimum number of characters to read)
    options.c_cc[VTIME] = 1; // always return after 0.1 seconds (time wait for data, tenth of seconds)

    // set the new options for the port
    tcsetattr(*(int *) handle, TCSAFLUSH, &options);
    
    return 0;
}

int writeData(void *handle, const char *buffer, int length){
    int n = write(*(int *) handle, buffer, length);
    if(n < 0){
        fprintf(stderr, "Error in serial write\n");
        return -1;
    }
    if(false){ // print or not
        printf("TX:");
        for(int i=0; i<length; ++i)
            printf(" %x", (unsigned char)(buffer[i]));
        printf("\n");
    }
    return n;
}

int readData(void *handle, char *buffer, int length){
    int bytes_read = read(*(int *) handle, buffer, length);
    if(bytes_read <= 0){
        return 0;
    }
    if(false){ // print or not
        printf("RX:");
        for(int i=0; i<bytes_read; ++i)
            printf(" %x", (unsigned char)(buffer[i]));
        printf("\n");
    }
    for(int i = 0; i < bytes_read; ++i){
        buffer[i] = 0x00FF & buffer[i];
    }
    return bytes_read;
}

