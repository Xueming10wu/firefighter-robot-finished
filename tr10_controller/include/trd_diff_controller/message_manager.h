#ifndef MESSAGE_MANAGER_H
#define MESSAGE_MANAGER_H

#include "stdint.h"
#include "trd_diff_controller/message.h"

class MessageManager{
public:
    MessageManager();
    int connect(const char *serial_port_name, const int baudrate);
    int disconnect();
    int sendMessage(void *handle, const char *buffer, int length);
    int rxMessage(void *handle, char *buffer, int length);
    // get
    int getEncoder();
    int getEncoderIMU();
    int getMotorStatus();
    // set
    void setSpeed(char speed_left, char speed_right);
    void setLRCalib();
    void setTimeout();
    void resetEncoder();
    void resetBase();
    int32_t encoder_left, encoder_right;
    int32_t encoder_left_offset, encoder_right_offset;
    bool first_time_flag;

    double imu_linear_accel_x, imu_linear_accel_y, imu_linear_accel_z; 
    double imu_angular_vel_x, imu_angular_vel_y, imu_angular_vel_z;
    double imu_orientation_x, imu_orientation_y, imu_orientation_z;
private:
    char serial_port_name[256];
    int baudrate;
    void *serial_handler;
    bool connect_flag;
    // send get msg
    SendMessageGetEncoder msg_get_encoder;
    SendMessageGetIMU msg_get_imu;
    SendMessageGetEncoderIMU msg_get_encoder_imu;
    // send set msg
    SendMessageSetSpeedLeft msg_set_speed_left;
    SendMessageSetSpeedRight msg_set_speed_right;
    SendMessageSetLRCalib msg_set_lr_calib;
    SendMessageSetTimeout msg_set_timeout;
    SendMessageResetEncoder msg_reset_encoder;
    SendMessageResetBase msg_reset_base;
    // rx msg
    RxMessage rx_message;
};

#endif
