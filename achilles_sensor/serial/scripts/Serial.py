#!/usr/bin/env python         
# -*- coding: utf-8 -*-

import serial 
import serial.tools.list_ports_linux
import threading
import time
import sys


class Serial:
    '串口类 提供 开关 设置 读写 功能'
    def __init__(self, name, baudrate):
        #self.PrintPortsList()
        self.name = name
        self.baudrate = baudrate
        self.setupPort()
        self.openPort()
        print ("初始化.....")
        time.sleep(1)
        print ("初始化完成")

    def info(self):
        print ("波特率(bps):%s"%self.__serialport.baudrate)
        print ("端口: %s\n"%self.__serialport.port)
   
    def PrintPortsList(self):
        port_list = list(serial.tools.list_ports_linux.comports())
        if len(port_list) == 0:
            print ("无可用串口")
        else:
            print ("------------------------------")
            print ("可用串口:")
            for i in range(0, len(port_list)):
                print (port_list[i])
            print ("------------------------------")
  
    def readThread(self):
        '循环接收数据，此为循环，线程实现，保存到缓存区中'
        print ("readThread")
        while self.__ReadFlag:
            if not self.__serialport.inWaiting():
                pass
            else:
                #print ("__RxUpdate ")
                #print (self.__RxUpdate)
                if (not self.__RxUpdate):
                    self.__readBuffer = self.__serialport.readline()
                    self.__RxUpdate = True
                else:
                    pass



    def setupPort(self):
        '串口协议为 (给定的端口名称 给定的波特率) 8位数据位 无校验位 1位停止位 不设置超时时间'
        try:
            self.__serialport = serial.Serial(self.name, self.baudrate)
            self.__serialport.writeTimeout = 2 #写超时
            self.info()
        except Exception as e:
            print ("串口被占用，请重新启动")
            
        
    def openPort(self):
        self.__readBuffer = "" #读取的数据
        self.__ReadFlag = True  #读取标志位
        self.__RxUpdate = False 
        try:
            if (self.__serialport.isOpen() == False):
                self.__serialport.open()
            self.rt = threading.Thread(target = self.readThread, args = ())
            self.rt.start()
            print ("%s已打开\n"%self.__serialport.name)
        except Exception as e:
            print ("串口打开失败，请检查串口是否存在")
            sys.exit()
        
    def closePort(self):
        "先关闭线程，再关闭串口"
        self.__ReadFlag = False
        self.rt.join()
        self.__serialport.close()

    def write(self, data):
        '写数据'
        if (data == ""):
            self.__serialport.write(" ")
            return 0
        else:
            #print ("数据发送成功")
            try:
                result = self.__serialport.write(data)
            except Exception as e:
                self.__serialport.flushOutput()
                self.closePort()
                self.__init__( self.name, self.baudrate )
            return result
    
    def read(self):
        '读数据'
        #print ("reading...")
        for i in range(0,3):
            #print ("wait ing..."),
            #print (i)
            time.sleep(i*i / 4)
            if self.__RxUpdate:
                #print ("第%d次等待"%i)
                self.__RxUpdate = False
                return self.__readBuffer
        self.__serialport.flushInput()
        self.closePort()
        self.__init__( self.name, self.baudrate )
        return "Timeout"


def main():
    serial = Serial("/dev/arduino_due", 9600)

if __name__ == '__main__':
    main()
