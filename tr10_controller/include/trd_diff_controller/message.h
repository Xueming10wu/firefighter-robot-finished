#ifndef MESSAGE_H
#define MESSAGE_H

char crc8(int size, char init_val, char *data);

#define SEND_MAXLEN 32
class SendMessage{
public:
    int len;
    char data[SEND_MAXLEN];
    void makeMsgValid();
};
class SendMessageGetEncoder : public SendMessage{
public:
    SendMessageGetEncoder();
};
class SendMessageGetIMU : public SendMessage{
public:
    SendMessageGetIMU();
};
class SendMessageGetEncoderIMU : public SendMessage{
public:
    SendMessageGetEncoderIMU();
};
class SendMessageSetSpeedLeft : public SendMessage{
public:
    SendMessageSetSpeedLeft();
    void loadSpeed(char c);
};
class SendMessageSetSpeedRight : public SendMessage{
public:
    SendMessageSetSpeedRight();
    void loadSpeed(char c);
};
class SendMessageSetLRCalib : public SendMessage{
public:
    SendMessageSetLRCalib();
};
class SendMessageSetTimeout : public SendMessage{
public:
    SendMessageSetTimeout();
};
class SendMessageResetEncoder : public SendMessage{
public:
    SendMessageResetEncoder();
};
class SendMessageResetBase : public SendMessage{
public:
    SendMessageResetBase();
};

#define RX_MAXLEN 256
class RxMessage{
public:
    RxMessage();
    RxMessage(const char *reply, const int n);
    int len;
    char data[RX_MAXLEN];
    bool isMsgValid();
    enum MsgType{
        ROGER = 5,
        Mode = 6,
        ERROR = 6,
        VC = 8,
        Encoder = 13,
        IMU = 23,
        EncoderIMU = 31
    };
};

#endif

