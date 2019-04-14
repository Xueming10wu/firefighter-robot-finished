#!/usr/bin/env python         
# -*- coding: utf-8 -*-

class MultCode:

    #各个字段的长度
    __fromeSize = 2
    __toSize = 2
    __numSize = 8
    __typeSize = 4
    __dataSize = 8
    __sumSize = __fromeSize + __toSize + __numSize + __typeSize + __dataSize

    def __init__(self):
        '初始化'
        #发送方
        self.__from = 0
        
        #接受方
        self.__to = 0

        #设备编号
        self.__num = 0

        #设备中的某个数据的类型编号
        self.__type = 0

        #数据
        self.__data = 0

        
        #保存传入数据
        self.__rxdata = " "
        #保存传出数据
        self.__txdata = " "

        #缓存区
        self.__tempBuffer = [" " for _ in range(self.__sumSize)]

        print ("协议转换器初始化完成")

    def __judgement(self, data, bit):
        '数据 位数'
        s = str(data)
        compare = 1
        for i in range(1,bit):
            compare = compare * 10
        
        for i in range(1,bit):
            if compare > data:
                s = "0" + s
            compare = compare / 10
        return s
    

    def __cut(self, begin, end):
        s = ""
        for i in range(begin, end):
            s = s + self.__tempBuffer[i]
        return s

    def setFrom(self, data):
        '设置 from 字段'
        self.__from = data
    
    def setTo(self, data):
        '设置 to 字段'
        self.__to = data
    
    def setNum(self, data):
        '设置 num 字段'
        self.__num = data

    def setType(self, data):
        '设置 type 字段'
        self.__type = data

    def setData(self, data):
        '设置 data 字段'
        self.__data = data

    def getFrom(self):
        '获取 from 字段'
        return self.__from

    def getTo(self):
        '获取 to 字段'
        return self.__to

    def getNum(self):
        '获取 num 字段'
        return self.__num

    def getType(self):
        '获取 type 字段'
        return self.__type

    def getData(self):
        '获取 data 字段'
        return self.__data
    
    def getSumSize(self):
        '获取标准协议总长度'
        return self.__sumSize
    


    def encode(self, from_, to_, num_, type_, data_):
        '不同字段的长度： form为2 to为2 num为8 type为4 data为8'
        _from = self.__judgement(from_, self.__fromeSize)
        _to = self.__judgement(to_, self.__toSize)
        _num = self.__judgement(num_, self.__numSize)
        _type = self.__judgement(type_, self.__typeSize)
        _data = self.__judgement(data_, self.__dataSize)
        
        self.__txdata = _from + _to + _num + _type + _data

        #print (self.__txdata)
        return self.__txdata

    def decode(self, data_):
        #print ("decoding...")
        self.__rxdata = data_
        self.__tempBuffer = list(data_)
        #print self.__tempBuffer
        position = 0
        
        self.setFrom(int(self.__cut(position, position + self.__fromeSize)))
        position += self.__fromeSize

        self.setTo(int(self.__cut(position, position + self.__toSize)))
        position += self.__toSize

        self.setNum(int(self.__cut(position, position + self.__numSize)))
        position += self.__numSize

        self.setType(int(self.__cut(position, position + self.__typeSize)))
        position += self.__typeSize

        self.setData(int(self.__cut(position, position + self.__dataSize)))
        position += self.__dataSize
        

        #print ("decode finished")