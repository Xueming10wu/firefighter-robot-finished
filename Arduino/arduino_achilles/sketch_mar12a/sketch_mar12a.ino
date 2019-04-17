/*
    '''
    字段    from_ to_ num_ type_ data_
    长度    2     2   8    4     8

    用来确定设备 编号 具体数据的处理

    From       00代表PC
            11代表arduino

    To         00代表PC
            11代表arduino

    Num     00000000~00009999为保留编号

            1111 0001 ~ 1111 0099为    mos管水泵，共99个
            1111 0101 ~ 1111 0199为    servo伺服舵机，共99个

            用法：
            使用前查看是否是保留编号，用于以后的紧急特权开发拓展
            然后通过 %1000 得到后三位数字， 后通过 /100留下的数字确定设备类型， %100确定具体是某个设备


    Type    设备自定义扩展

    data    设备自定义扩展
    '''
  mos管端
    Num段        Type段         data段           说明
    开关号        0001          无效00           关闭
    开关号        0002          无效00           打开

  舵机端
    Num段        Type段         data段           说明
    舵机号        0001          脉宽+时间!        写角度 脉宽为4位，时间为4位
    舵机号        0003          无效         读角度
    舵机号   0003    返回4位脉宽   读角度响应



  此代码部署下位机
*/

#include <Equipment.h>
#include <MultCode.h>
#include <Arduino.h>


void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
}


int main()
{
  init();
  setup();

  //编码器
  MultCode multcode = MultCode();

  //编码缓存区
  char *txBuffer = new char[multcode.getSumSize()];

  //解码缓存区
  char *rxBuffer = new char[multcode.getSumSize()];

  //数据更新标志位
  bool dataUpdate = false;

  //大循环标志位
  bool startOrStop = true;


  MosSwitch mos1 = MosSwitch(23, 12);
  //设置舵机端口
  SerialServo::setPort( 1 );
  SerialServo servo1 = SerialServo();
  SerialServo servo2 = SerialServo();
  //舵机编号
  servo1.num_ = 1;
  servo2.num_ = 2;

  while ( startOrStop )
  {
    /****************************读数据开始****************************/
    if (Serial.available())
    {
      for ( int i = 0; i < multcode.getSumSize(); i ++)
      {
        //清理之前BUFFER里面的缓存
        rxBuffer[i] = 0;
      }

      for ( int i = 0 ; Serial.available() && i < multcode.getSumSize(); i ++)
      {
        rxBuffer[i] = Serial.read();

        //进行串口缓存恢复
        delay(5);
      }

      //数据更新标志
      dataUpdate = true;

      //Serial.println(rxBuffer);
    }

    //处理数据
    if (dataUpdate)
    {
      dataUpdate = false;
      multcode.decode(rxBuffer);
    }

    int from = multcode.getFrom();
    int to = multcode.getTo();
    int num = multcode.getNum();
    int type = multcode.getType();
    int data = multcode.getData();

    startOrStop = true;

    if (from * 100 + to == 11)
    { //如果数据从上位机来了
      switch ( (num % 1000) / 100 )
      {
        case 0:
          /*******************
                MOS管开关
          ********************/
          //Serial.print("选中MOS管类型   : ");
          //std::cout << "选中MOS管类型   : " ;
          switch (num % 100)
          {
            case 1:
              //Serial.print("1 号开关 : ");
              //std::cout << "1 号开关 : ";
              switch (type)
              {
                case 1:
                  //Serial.println("关功能 ");
                  //std::cout << "关功能 " << std::endl;
                  mos1.Close();
                  break;
                case 2:
                  //Serial.println("开功能 ");
                  //std::cout << "开功能 " << std::endl;
                  mos1.Open();
                  break;
                default:
                  //Serial.println("功能缺失");
                  //std::cout << "功能缺失" << std::endl;
                  delay(5);
              }
              break;

            default:
              //Serial.println("无此开关");
              //std::cout << "无此开关" << std::endl;
              delay(5);
          }

          break;
        case 1:
          /*******************
                串口舵机
          ********************/
          //Serial.print("选中串口舵机类型");
          //std::cout << "选中串口舵机类型 : " ;
          switch (num % 100)
          {
            case 1:
              //Serial.print("1 号舵机 : ");
              //std::cout << "1 号舵机 : ";
              switch (type)
              {
                case 1:
                  //Serial.println("写功能 ");
                  //std::cout << "写功能 " << std::endl;
                  servo1.write( num % 100, data / 10000, data % 10000 );
                  break;
                case 3:
                  //Serial.println("读功能 ");
                  //std::cout << "读功能 " << std::endl;
                  servo1.read( num % 100 );
                  break;
                default:
                  //Serial.println("功能缺失");
                  //std::cout << "功能缺失" << std::endl;
                  delay(5);
              }
              break;

            case 2:
              //Serial.print("2 号舵机 : ");
              //std::cout << "2 号舵机 : ";
              switch (type)
              {
                case 1:
                  //Serial.println("写功能");
                  //std::cout << "写功能 " << std::endl;
                  servo2.write( num % 100, data / 10000, data % 10000 );
                  break;
                case 3:
                  //Serial.println("读功能");
                  //std::cout << "读功能 " << std::endl;
                  servo2.read( num % 100 );
                  break;
                default:
                  //Serial.println("功能缺失");
                  //std::cout << "功能缺失" << std::endl;
                  delay(5);
              }
              break;

            default:
              //Serial.println("无此舵机");
              //std::cout << "无此舵机" << std::endl;
              //未来可扩展舵机，当前无对应编号
              delay(5);
          }
          break;
        default:
          /*******************
                错误
          ********************/
          delay(5);
          //Serial.println("无此种设备");
          //std::cout << "无此种设备" << std::endl;
          //未来扩展设备部分，当前无对应编号
      }
    }
    /****************************读数据结束****************************/
    delay(50);
    /****************************写数据开始****************************/
    //servo角度数据发送
    if( servo1.from_ * 100 + servo1.to_ == 1100 )
    {
      //Serial.println("sead respond");
      multcode.encode( servo1.from_, servo1.to_, servo1.num_, servo1.type_, servo1.data_ );
      for( int i = 0 ; i < multcode.getSumSize(); i ++)
      {
        //由于encode将组合好后的代码放入txData数组中，为了不出现内存泄露的风险，
        //直接通过get访问并直接通过串口发送
        Serial.write( * (multcode.getTxData() + i));
      }
      Serial.write("\n");
      //恢复与清理
      servo1.from_ = 0;
      servo1.to_ = 0;
    }

    if( servo2.from_ * 100 + servo2.to_ == 1100 )
    {
      //Serial.println("sead respond");
      multcode.encode( servo2.from_, servo2.to_, servo2.num_, servo2.type_, servo2.data_ );
      for( int i = 0 ; i < multcode.getSumSize(); i ++)
      {
        //由于encode将组合好后的代码放入txData数组中，为了不出现内存泄露的风险，
        //直接通过get访问并直接通过串口发送
        Serial.write( * (multcode.getTxData() + i));
      }
      Serial.write("\n");
      //恢复与清理
      servo2.from_ = 0;
      servo2.to_ = 0;
    }

    //恢复与清理
    from = 0;
    to = 0;
    num = 0;
    type = 0;
    data = 0;
    
    /****************************写数据结束****************************/
    delay(50);
  }
  Serial.println("Arduino ready poweroff !");
  return 0;
}
