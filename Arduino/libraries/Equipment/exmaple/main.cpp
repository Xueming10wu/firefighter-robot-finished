#include <iostream>
#include "Equipment.h"

using namespace std;

int main()
{
    //串口接收
	
    //解码
    int num = 11110002;
    int type = 3;
    int data = 32665488;

    //
    //MosSwitch sw = MosSwitch(23, 12);
    //设置舵机端口以及编号
    SerialServo::setPort( 1 );
    SerialServo servo1 = SerialServo();
    SerialServo servo2 = SerialServo();
    servo1.num_ = 1;
    servo2.num_ = 2;

    switch( (num%1000) / 100 )
    {
    case 0:
        /*******************
        *     MOS管开关     *
        ********************/
        std::cout << "选中MOS管类型   : " ;
        switch(num%100)
        {
            case 1:
                std::cout << "1 号开关 : ";
                switch(type)
                {
                    case 1:
                        std::cout << "关功能 " << std::endl;
                        //mos1.Close();
                        break;
                    case 2:
                        std::cout << "开功能 " << std::endl;
                        //mos1.Open();
                        break;
                    default:
                        std::cout << "功能缺失" << std::endl;

                }
                break;

            default:
                std::cout << "无此开关" << std::endl;
        }

        break;
    case 1:
        /*******************
        *     串口舵机      *
        ********************/
        std::cout << "选中串口舵机类型 : " ;
        switch(num%100)
        {
            case 1:
                std::cout << "1 号舵机 : ";
                switch(type)
                {
                    case 1:
                        std::cout << "写功能 " << std::endl;
                        servo1.write( num%100, data / 10000, data % 10000 );
                        break;
                    case 3:
                        std::cout << "读功能 " << std::endl;
                        servo1.read( num%100 );
                        break;
                    default:
                        std::cout << "功能缺失" << std::endl;

                }
                break;

            case 2:
                std::cout << "2 号舵机 : ";
                switch(type)
                {
                    case 1:
                        std::cout << "写功能 " << std::endl;
                        servo2.write( num%100, data / 10000, data % 10000 );
                        break;
                    case 3:
                        std::cout << "读功能 " << std::endl;
                        servo2.read( num%100 );
                        break;
                    default:
                        std::cout << "功能缺失" << std::endl;
                }
                break;

            default:
                std::cout << "无此舵机" << std::endl;
            //未来可扩展舵机，当前无对应编号
        }
        break;
    default:
        /*******************
        *     错误          *
        ********************/
        std::cout << "无此种设备" << std::endl;
        //未来扩展设备部分，当前无对应编号
    }


    //char *p = s.intToChar(6842);
    //int i = s.charToInt(p,4);
    //cout << p << endl;
    //cout << i << endl;
    //s.read(35);
    //s.write( 32,1875,120);
    //s.info();

    return 0;
}
