#ifndef EQUIPMENT_H
#define EQUIPMENT_H

#include <Arduino.h>

/************************
 *       抽象设备        *
*************************/
class Equipment
{
public:
    //发送方
    int from_;

    //接受方
    int to_;

    //设备编号
    int num_;

    //设备中的某个数据的类型编号
    int type_;

    //数据
    int data_;

    //构造函数
    Equipment();

    //显示信息
    void info() const;

    //不填补0
    char * intToChar( int data ) const;

    //数组指针和数组的长度
    int charToInt( char * str, int length) const;

    //恢复并清空
    void clear();
};

/************************
 *       具体设备        *
*************************/
class SerialServo : public Equipment
{
public:
    SerialServo();
    void info() const;

    void read(int servo_id);
    void write(int servo_id, int width, int duration);

    void static setPort( int port_id ){ serialPort = port_id; };
    int static getPort(){ return serialPort; };

private:
    //界符
    char start_flag;
    char stop_flag;
    char servo_id_flag;
    char servo_duration_flag;

    //串口号 取值 0到3
    static int serialPort;

    //读取成功时调用，为read函数服务
    void readSuccess( char * respond );
};


class MosSwitch : public Equipment
{
  public:
    MosSwitch(const int& pwm_port, const int& port_digital );
    void Open();
    void Close();

  private:
    const int MosSwitch_port_pwm;
    const int MosSwitch_port_digital;
};



#endif // EQUIPMENT_H
