#include "trd_diff_controller/message_manager.h"
#include "trd_diff_controller/serial_port.h"

#include "ros/ros.h"

MessageManager::MessageManager(){
    connect_flag = false;
    first_time_flag = true;
    encoder_left_offset = 0;
    encoder_right_offset = 0;
}
int MessageManager::connect(const char *serial_port_name, const int baudrate){
    sprintf(this->serial_port_name, "%s", serial_port_name);
    this->baudrate = baudrate;
    if(connect_flag){
        return -1;
    }
    if(openSerialPort(&serial_handler, serial_port_name) < 0){
        return -2;
    }
    if(setupSerialPort(serial_handler) < 0){
        return -3;
    }
    connect_flag = true;
    return 0;
}

int MessageManager::sendMessage(void *handle, const char *buffer, int length){
    int n = writeData(handle, buffer, length);
    if(n == length){
        return n;
    }
    connect_flag = false;
    while(true){ // reconnect
        usleep(1000*1000);
        ROS_WARN("Reconnecting to serial port: %s...", this->serial_port_name);
        if(0 == connect(this->serial_port_name, this->baudrate)){
            ROS_WARN("Connected!");
            return 0;
        }
    }
}

int MessageManager::rxMessage(void *handle, char *buffer, int length){
    int n = readData(handle, buffer, length);
    if(n == length){
        return n;
    }
    connect_flag = false;
    while(true){ // reconnect
        usleep(1000*1000);
        ROS_WARN("Reconnecting to serial port: %s...", this->serial_port_name);
        if(0 == connect(this->serial_port_name, this->baudrate)){
            ROS_WARN("Connected!");
            return 0;
        }
    }
}

int MessageManager::disconnect(){
    closeSerialPort(serial_handler);
    return 0;
}

int MessageManager::getEncoder(){
    sendMessage(serial_handler, msg_get_encoder.data, msg_get_encoder.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.Encoder);
    if(rx_message.len == rx_message.Encoder && rx_message.isMsgValid()){
        encoder_left   = (rx_message.data[3]&0xFF) << 24;
        encoder_left  |= (rx_message.data[4]&0xFF) << 16;
        encoder_left  |= (rx_message.data[5]&0xFF) << 8;
        encoder_left  |= (rx_message.data[6]&0xFF);
        encoder_right  = (rx_message.data[7]&0xFF) << 24;
        encoder_right |= (rx_message.data[8]&0xFF) << 16;
        encoder_right |= (rx_message.data[9]&0xFF) << 8;
        encoder_right |= (rx_message.data[10]&0xFF);
        if(first_time_flag){
            encoder_left_offset = encoder_left;
            encoder_right_offset = encoder_right;
            ROS_INFO("Encoder offset left: %d, right: %d", encoder_left_offset, encoder_right_offset);
            first_time_flag = false;
        }
        encoder_left -= encoder_left_offset;
        encoder_right -= encoder_right_offset;
        return 0;
    }
    return -1;
}

int MessageManager::getEncoderIMU(){
    sendMessage(serial_handler, msg_get_encoder_imu.data, msg_get_encoder_imu.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.EncoderIMU);
    if(rx_message.len == rx_message.EncoderIMU && rx_message.isMsgValid()){
        encoder_left   = (rx_message.data[3]&0xFF) << 24;
        encoder_left  |= (rx_message.data[4]&0xFF) << 16;
        encoder_left  |= (rx_message.data[5]&0xFF) << 8;
        encoder_left  |= (rx_message.data[6]&0xFF);
        encoder_right  = (rx_message.data[7]&0xFF) << 24;
        encoder_right |= (rx_message.data[8]&0xFF) << 16;
        encoder_right |= (rx_message.data[9]&0xFF) << 8;
        encoder_right |= (rx_message.data[10]&0xFF);
        if(first_time_flag){
            encoder_left_offset = encoder_left;
            encoder_right_offset = encoder_right;
            ROS_INFO("Encoder offset left: %d, right: %d", encoder_left_offset, encoder_right_offset);
            first_time_flag = false;
        }
        encoder_left -= encoder_left_offset;
        encoder_right -= encoder_right_offset;
        int16_t imu_tmp;
        imu_tmp  = (rx_message.data[11]&0xFF) << 8;
        imu_tmp |= (rx_message.data[12]&0xFF);
        imu_angular_vel_x = 2000.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[13]&0xFF) << 8;
        imu_tmp |= (rx_message.data[14]&0xFF);
        imu_angular_vel_y = 2000.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[15]&0xFF) << 8;
        imu_tmp |= (rx_message.data[16]&0xFF);
        imu_angular_vel_z = 2000.0*imu_tmp / 32768;

        imu_tmp  = (rx_message.data[17]&0xFF) << 8;
        imu_tmp |= (rx_message.data[18]&0xFF);
        imu_linear_accel_x = 16.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[19]&0xFF) << 8;
        imu_tmp |= (rx_message.data[20]&0xFF);
        imu_linear_accel_y = 16.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[21]&0xFF) << 8;
        imu_tmp |= (rx_message.data[22]&0xFF);
        imu_linear_accel_z = 16.0*imu_tmp / 32768;

        imu_tmp  = (rx_message.data[23]&0xFF) << 8;
        imu_tmp |= (rx_message.data[24]&0xFF);
        imu_orientation_x = 1.0*imu_tmp / 100;
        imu_tmp  = (rx_message.data[25]&0xFF) << 8;
        imu_tmp |= (rx_message.data[26]&0xFF);
        imu_orientation_y = 1.0*imu_tmp / 100;
        imu_tmp  = (rx_message.data[27]&0xFF) << 8;
        imu_tmp |= (rx_message.data[28]&0xFF);
        imu_orientation_z = 1.0*imu_tmp / 100;
        return 0;
    }
    return -1;
}
int MessageManager::getMotorStatus(){
    SendMessage send_msg;
    // version
    send_msg.len = 5;
    send_msg.data[2] = 0x29;
    send_msg.makeMsgValid();
    sendMessage(serial_handler, send_msg.data, send_msg.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, 6);
    if(rx_message.len == 6 && rx_message.isMsgValid()){
        ROS_INFO("TRD Version: %x", rx_message.data[3]);
    }
    // Mode
    send_msg.len = 5;
    send_msg.data[2] = 0x2B;
    send_msg.makeMsgValid();
    sendMessage(serial_handler, send_msg.data, send_msg.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, 6);
    if(rx_message.len == 6 && rx_message.isMsgValid()){
        ROS_INFO("Mode: %x", rx_message.data[3]);
    }
    // Error Code
    send_msg.len = 5;
    send_msg.data[2] = 0x2D;
    send_msg.makeMsgValid();
    sendMessage(serial_handler, send_msg.data, send_msg.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, 6);
    if(rx_message.len == 6 && rx_message.isMsgValid()){
        ROS_INFO("Error Code: %x", rx_message.data[3]);
    }
    // voltage
    send_msg.len = 5;
    send_msg.data[2] = 0x26;
    send_msg.makeMsgValid();
    sendMessage(serial_handler, send_msg.data, send_msg.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, 6);
    if(rx_message.len == 6 && rx_message.isMsgValid()){
        ROS_INFO("Voltage: 0x%x(%d)", rx_message.data[3], rx_message.data[3]);
    }
    return -1;
}

void MessageManager::setSpeed(char speed_left, char speed_right){
    msg_set_speed_left.loadSpeed(speed_left);
    msg_set_speed_right.loadSpeed(speed_right);
    sendMessage(serial_handler, msg_set_speed_left.data, msg_set_speed_left.len);
    usleep(1000);
    sendMessage(serial_handler, msg_set_speed_right.data, msg_set_speed_right.len);
}
void MessageManager::setLRCalib(){
    sendMessage(serial_handler, msg_set_lr_calib.data, msg_set_lr_calib.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.ROGER);
    ROS_INFO("rx lr calib len: %d", rx_message.len);
    if(rx_message.len == rx_message.ROGER && rx_message.isMsgValid()){
        ROS_INFO("Set LR Calib OK");
    }
    else{
        ROS_WARN("Set LR Calib falied");
    }
}
void MessageManager::setTimeout(){
    sendMessage(serial_handler, msg_set_timeout.data, msg_set_timeout.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.ROGER);
    ROS_INFO("rx timeout len: %d", rx_message.len);
    if(rx_message.len == rx_message.ROGER && rx_message.isMsgValid()){
        ROS_INFO("Set timeout OK");
    }
    else{
        ROS_WARN("Set timeout falied");
    }
}
void MessageManager::resetEncoder(){
    sendMessage(serial_handler, msg_reset_encoder.data, msg_reset_encoder.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.ROGER);
    ROS_INFO("rx reset encoder len: %d", rx_message.len);
    if(rx_message.len == rx_message.ROGER && rx_message.isMsgValid()){
        ROS_INFO("Reset encoder OK");
    }
    else{
        ROS_WARN("Reset encoder falied");
    }
}
void MessageManager::resetBase(){
    sendMessage(serial_handler, msg_reset_base.data, msg_reset_base.len);
    usleep(50000);
    rx_message.len = rxMessage(serial_handler, rx_message.data, rx_message.ROGER);
    ROS_INFO("rx reset base len: %d", rx_message.len);
    if(rx_message.len == rx_message.ROGER && rx_message.isMsgValid()){
        ROS_INFO("Reset base OK");
    }
    else{
        ROS_WARN("Reset base falied");
    }
}

