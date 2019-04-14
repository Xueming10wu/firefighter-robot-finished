#include "trd_diff_controller/message.h"

char crc8(int size, char init_val, char *data){
    char crc = init_val;
    while(size--)
    {
        crc = crc ^ *data++;
    }
    return crc;
}
void SendMessage::makeMsgValid()
{
    if(len < SEND_MAXLEN)
    {
        data[0] = 0xEA;
        data[1] = len - 2;
        data[len-1] = 0x0d;
        data[len-2] = crc8(len-2, 0x00, data);
    }
}
SendMessageGetEncoder::SendMessageGetEncoder(){
    len = 5;
    data[2] = 0x25;
    makeMsgValid();
}
SendMessageGetIMU::SendMessageGetIMU(){
    len = 5;
    data[2] = 0x45;
    makeMsgValid();
}
SendMessageGetEncoderIMU::SendMessageGetEncoderIMU(){
    len = 5;
    data[2] = 0x46;
    makeMsgValid();
}
SendMessageSetSpeedLeft::SendMessageSetSpeedLeft(){
    len = 6;
    data[2] = 0x31;
    data[3] = 0x00;
    makeMsgValid();
}
void SendMessageSetSpeedLeft::loadSpeed(char c){
    data[3] = c;
    makeMsgValid();
}
SendMessageSetSpeedRight::SendMessageSetSpeedRight(){
    len = 6;
    data[2] = 0x32;
    data[3] = 0x00;
    makeMsgValid();
}
void SendMessageSetSpeedRight::loadSpeed(char c){
    data[3] = c;
    makeMsgValid();
}
SendMessageSetLRCalib::SendMessageSetLRCalib(){
    len = 5;
    data[2] = 0x37;
    makeMsgValid();
}
SendMessageSetTimeout::SendMessageSetTimeout(){
    len = 5;
    data[2] = 0x39;
    makeMsgValid();
}
SendMessageResetEncoder::SendMessageResetEncoder(){
    len = 5;
    data[2] = 0x35;
    makeMsgValid();
}
SendMessageResetBase::SendMessageResetBase(){
    len = 5;
    data[2] = 0x50;
    makeMsgValid();
}

RxMessage::RxMessage(){}
RxMessage::RxMessage(const char *reply, const int n){
    len = n;
    if(n>0 && n<RX_MAXLEN){
        for(int i = 0; i < n; ++i){
            data[i] = reply[i];
        }
    }
}
bool RxMessage::isMsgValid(){
    if(len > 0 && len < RX_MAXLEN){
        if(data[0]==char(0xEA) && data[1]==len-2 && crc8(len-2, 0x00, data)==data[len-2]){
            return true;
        }
    }
    return false;
}

