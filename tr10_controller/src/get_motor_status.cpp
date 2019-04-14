#include "trd_diff_controller/message_manager.h"
#include "stdlib.h"
#include <string>

int main(int argc, char* argv[]){
    std::string serialport_name = "/dev/motor_trd";
    int baudrate = 38400;
    if( 2 == argc){
       serialport_name = argv[1];
    }
    else if (3 == argc){
       serialport_name = argv[1];
       baudrate = atoi(argv[2]);
    }
    MessageManager message_manager;
    if(message_manager.connect(serialport_name.c_str(), baudrate) < 0){
        return -1;
    }
    message_manager.getMotorStatus();
    return 0;
}

