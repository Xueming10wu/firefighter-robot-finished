#include "Equipment.h"
#include <Arduino.h>

/************************
 *       抽象设备        *
*************************/

Equipment::Equipment()
{
    from_ = 0;
    to_= 0;
    num_= 0;
    type_= 0;
    data_= 0;
}

void Equipment::info() const
{
    /*
    std::cout << "\nfrom:" << from_
              << ",   to:" << to_
              << ",   num:" << num_
              << ",   type:" << type_
              << ",   data:" << data_<< std::endl;
    */
};

char* Equipment::intToChar(int data) const
{
    //std::cout << "intToChar()" << std::endl;
    int bit;
    int temp = data;

    for( bit = 0; temp > 0; bit ++ )
    {
        temp = temp / 10;
    }
    //std::cout << bit << std::endl;
    char *s = new char[bit];

    int remind = 0;

    for(int i = 0; data > 0; i ++)
    {
        remind = data % 10;
        data = data / 10;
        s[bit - i - 1] = (char)(remind + 48);
    }
    return s;
}


int Equipment::charToInt(char* str, int length) const
{
    int num = 0;
    char char_temp;
    int int_temp;
    for(int i = 0 ; i < length; i ++)
    {
        char_temp = str[i];
        int_temp = (int)char_temp -48;
        num += int_temp;
        num *= 10;
    }
    num = num / 10;
    return num;
}

void Equipment::clear()
{
    Equipment();
}

/************************
 *       具体设备        *
*************************/
//默认为1
int SerialServo::serialPort = 1;

SerialServo::SerialServo() : Equipment()
{
    //std::cout << "SerialServo()" << std::endl;
    start_flag = '#';
    stop_flag = '!';
    servo_id_flag = 'P';
    servo_duration_flag = 'T';
    setPort(1);
}

void SerialServo::info() const
{
    Equipment::info();
    //std::cout << "SerialPort: Serial" << serialPort <<std::endl;
}


void SerialServo::read( int servo_id )
{
    //#?PRAD! 串口舵机读协议规则
    //delay(5);
    //Serial.println(servo_id);
    int temp = servo_id;
    int len = 6;
    int data_size = 0;
    char * c_sid = intToChar(servo_id);
    for( ; servo_id > 0 ; len ++)
    {
        servo_id = servo_id / 10;
        data_size ++;
    }
    //std::cout << len << std::endl;

    //request为 arduino->串口舵机 请求读 格式
    //respond为 串口舵机->arduino 响应读 格式
    char * request = new char[len];
    char * respond = new char[10];

    request[0] = start_flag;
    for( int i = 0; i < data_size; i++)
    {
        request[i + 1] = c_sid[i];
    }
    request[len-5] = servo_id_flag;
    request[len-4] = 'R';
    request[len-3] = 'A';
    request[len-2] = 'D';
    request[len-1] = stop_flag;
    //std::cout << "request : " <<request << std::endl;
    //std::cout << "respond : " <<respond << std::endl;

    /*模拟读取串口，实际可以 根据serialPort 来确定 接收数据，把respond字段填满，把下面的switch打开即可
    respond[0] = '#'; respond[1] = '0'; respond[2] = '0'; respond[3] = '2'; respond[4] = 'P';
    respond[5] = '0'; respond[6] = '9'; respond[7] = '8'; respond[8] = '0'; respond[9] = '!';
    */
    switch(serialPort)
    {
    case 0:
        for( int i = 0 ; i < len; i ++)
        {
            Serial.write(request[i]);
        }
        
        //for( int count = 0 ; count <= 10; count++)
        //{
            delay( 50 );
            if (Serial.available())
            {
                for ( int i = 0; i < 10; i ++)
                {
                    //清理之前BUFFER里面的缓存
                    respond[i] = 0;
                }

                for ( int i = 0; Serial.available(); i ++)
                {
                    respond[i] = Serial.read();

                    //进行串口缓存恢复
                    delay(5);
                }
                //五个字段设置
                //然后通过访问字段进行编码 >> 串口发送到上位机 >> 释放encode内存
                readSuccess( respond );
                //break;
            }
            else
            //if( count >= 10)
            {
                Serial.println("servo read timeout");
            }
        //}
        break;
    case 1:
        for( int i = 0 ; i < len; i ++)
        {
            Serial1.write(request[i]);
        }
        
        //for( int count = 1 ; count < 50; count++)
        //{
            delay( 50 );
            if (Serial1.available())
            {
                for ( int i = 0; i < 10; i ++)
                {
                    //清理之前BUFFER里面的缓存
                    respond[i] = 0;
                }

                for ( int i = 0; Serial1.available(); i ++)
                {
                    respond[i] = Serial1.read();

                    //进行串口缓存恢复
                    delay(5);
                }
                //五个字段设置
                //然后通过访问字段进行编码 >> 串口发送到上位机 >> 释放encode内存
                readSuccess( respond );
                break;
            }
            else
            //if( count >= 49)
            {
                Serial.println("servo read timeout");
            }
        //}
        break;

    case 2:
        for( int i = 0 ; i < len; i ++)
        {
            Serial2.write(request[i]);
        }
        
        //for( int count = 1 ; count < 50; count++)
        //{
            delay( 50 );
            if (Serial2.available())
            {
                for ( int i = 0; i < 10; i ++)
                {
                    //清理之前BUFFER里面的缓存
                    respond[i] = 0;
                }

                for ( int i = 0; Serial2.available(); i ++)
                {
                    respond[i] = Serial2.read();

                    //进行串口缓存恢复
                    delay(5);
                }
                //五个字段设置
                //然后通过访问字段进行编码 >> 串口发送到上位机 >> 释放encode内存
                readSuccess( respond );
                break;
            }
            else
            //if( count >= 49)
            {
                Serial.println("servo read timeout");
            }
        //}
        break;

    case 3:
        for( int i = 0 ; i < len; i ++)
        {
            Serial3.write(request[i]);
        }
        
        //for( int count = 1 ; count < 50; count++)
        //{
            delay( 50 );
            if (Serial3.available())
            {
                for ( int i = 0; i < 10; i ++)
                {
                    //清理之前BUFFER里面的缓存
                    respond[i] = 0;
                }

                for ( int i = 0; Serial3.available(); i ++)
                {
                    respond[i] = Serial3.read();

                    //进行串口缓存恢复
                    delay(5);
                }
                //五个字段设置
                //然后通过访问字段进行编码 >> 串口发送到上位机 >> 释放encode内存
                readSuccess( respond );
                break;
            }
            else
            //if( count >= 49)
            {
                Serial.println("servo read timeout");
            }
        //}
        break;

    default:
        for( int i = 0 ; i < len; i ++)
        {
            Serial.write(request[i]);
        }
        
        //for( int count = 1 ; count < 50; count++)
        //{
            delay( 50 );
            if (Serial.available())
            {
                for ( int i = 0; i < 10; i ++)
                {
                    //清理之前BUFFER里面的缓存
                    respond[i] = 0;
                }

                for ( int i = 0; Serial.available(); i ++)
                {
                    respond[i] = Serial.read();

                    //进行串口缓存恢复
                    delay(5);
                }
                //五个字段设置
                //然后通过访问字段进行编码 >> 串口发送到上位机 >> 释放encode内存
                readSuccess( respond );
                break;
            }
            else
            //if( count >= 49)
            {
                Serial.println("servo read timeout");
            }
        //}
        break;
    }
    
    if( num_ != temp)
    {//后续可以对 respond
        //std::cout << "Different id" << std::endl;
        //Serial.println("Different id");
    }

    //std::cout << "num_ : " << num_ << std::endl;
    //std::cout << "data_ : " << data_ << std::endl;

    delete c_sid;
    delete request;
    delete respond;
}

void SerialServo::write( int servo_id, int width, int duration )
{
    int len = 4;
    int id_size = 0;
    int width_size = 0;
    int duration_size = 0;

    char * c_sid = intToChar( servo_id );
    char * c_width = intToChar( width );
    char * c_duration = intToChar( duration );

    for( ; servo_id > 0 ; id_size ++)
    {
        servo_id = servo_id / 10;
    }

    for( ; width > 0 ; width_size ++)
    {
        width = width / 10;
    }

    for( ; duration > 0 ; duration_size ++)
    {
        duration = duration / 10;
    }

    char * cmd = new char[ len + id_size + width_size + duration_size ];
    cmd[0] = start_flag;
    for( int i = 0; i < id_size; i++)
    {
        cmd[i + 1] = c_sid[i];
    }
    cmd[1 + id_size] = servo_id_flag;
    for( int i = 0; i < width_size; i++)
    {
        cmd[i + 2 + id_size] = c_width[i];
    }
    cmd[2 + id_size + width_size] = servo_duration_flag;
    for( int i = 0; i < duration_size; i++)
    {
        cmd[i + 3 + id_size + width_size] = c_duration[i];
    }
    cmd[3 + id_size + width_size + duration_size] = stop_flag;

    //std::cout << cmd << std::endl;
    //命令写入舵机
    switch(serialPort)
    {
    case 0:
        for(int i = 0 ; i < len + id_size + width_size + duration_size ; i ++)
        {
            Serial.write(cmd[i]);
        }
        break;

    case 1:
        for(int i = 0 ; i < len + id_size + width_size + duration_size ; i ++)
        {
            Serial1.write(cmd[i]);
        }
        break;

    case 2:
        for(int i = 0 ; i < len + id_size + width_size + duration_size ; i ++)
        {
            Serial2.write(cmd[i]);
        }
        break;

    case 3:
        for(int i = 0 ; i < len + id_size + width_size + duration_size ; i ++)
        {
            Serial3.write(cmd[i]);
        }
        break;

    default:
        for(int i = 0 ; i < len + id_size + width_size + duration_size ; i ++)
        {
            Serial1.write(cmd[i]);
        }
    }
    
    delete c_sid;
    delete c_width;
    delete c_duration;
    delete cmd;
}

void SerialServo::readSuccess( char * respond )
{
    from_= 11;
    to_ = 0;
    type_ = 3;
    //符合协议中对舵机编号域的限定
    num_ = charToInt( respond + 1 , 3 ) + 11110100;
    data_ = charToInt( respond + 5 , 4 );
    //Serial.println("readSuccess");
}


MosSwitch::MosSwitch(const int& pwm_port , const int& port_digital )
: Equipment(),
  MosSwitch_port_pwm ( pwm_port ),
  MosSwitch_port_digital ( port_digital )
{
  Close();
  //Serial.flush();
  //Serial.println("MosSwitch constructing...");

};

void MosSwitch::Open()
{

  Serial.println("MosSwitch    : Open...");
  //Serial.println("MosSwitch_port_pwm is UP");
  digitalWrite(MosSwitch_port_pwm , HIGH );
  //此时输出高电压

  //Serial.println("MosSwitch_port_digital is DOWN");
  digitalWrite(MosSwitch_port_digital , LOW);
  //Serial.println();

};

void MosSwitch::Close()
{

  //Serial.println("MosSwitch    : Close...");
  //Serial.println("MosSwitch_port_pwm is DOWN");
  digitalWrite(MosSwitch_port_pwm , LOW);
  //此时依然输出高电压

  //Serial.println("MosSwitch_port_digital is UP");
  digitalWrite(MosSwitch_port_digital , HIGH);
  //此时输出低电压
  //Serial.println();

};

