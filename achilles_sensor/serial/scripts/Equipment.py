#!/usr/bin/env python         
# -*- coding: utf-8 -*-


class Equipment(object):
    '''
    字段    from_ to_ num_ type_ data_
    长度    2     2   8    4     8
        
    用来确定设备 编号 具体数据的处理

    From       00代表PC
            11代表arduino

    To         00代表PC
            11代表arduino

    Num     00000000~00009999为保留编号

            1111 0000 ~ 1111 0099为    mos管水泵，共100个
            1111 0100 ~ 1111 0199为    串口servo伺服舵机，共100个

            用法：
            使用前查看是否是保留编号，用于以后的紧急特权开发拓展
            然后通过 %1000 得到后三位数字， 后通过 /100留下的数字确定设备类型， %100确定具体是某个设备
        

    Type    设备自定义扩展

    data    设备自定义扩展
    '''

    def __init__(self):
        '初始化'
        #发送方
        self.from_ = 0
        #接受方
        self.to_ = 0
        #设备编号
        self.num_ = 0
        #设备中的某个数据的类型编号
        self.type_ = 0
        #数据
        self.data_ = 0
    
    def info(self):
        print ("from_ %d"%self.from_)
        print ("to_ %d"%self.to_)
        print ("num_ %d"%self.num_)
        print ("type_ %d"%self.type_)
        print ("data_ %d"%self.data_)
    
    def request(self):
        '''
        PC->Arduino通用请求
        '''
        self.from_ = 0
        self.to_ = 11
    
    def respond(self):
        '''
        PC->Arduino通用响应
        '''
        self.from_ = 11
        self.to_ = 0
    
    

class SerialServo(Equipment):
    '''
    舵机设备
    '''

    '''
    舵机取值范围为 1111 0101 ~ 1111 0199 共99个。记录开始值。对应设备 1~99号舵机
    '''
    id_start = 11110100

    '''
    真正控制舵机的脉宽(PWM/PDM)参数
    '''
    minWidth = 500
    maxWidth = 2500
    WidthField = 2500 - 500

    '''
    逻辑角度，注意 没有符号控制，即没有负角度的出现。
    若做 机器人坐标变换，可以统一将 这里角度-90=逻辑坐标角度
    '''
    minAngle = 0
    maxAngle = 180
    AngleField = 180
    '''
    '''
    
    def __init__(self):
        Equipment.__init__(self)
    
    def angleToWidth( self, angle ):
        '''
        角度转脉宽
        '''
        width = int(angle * self.WidthField / self.AngleField) +  self.minWidth
        return width

    def widthToAngle( self, width ):
        '''
        脉宽转角度
        '''
        angle = int((width - self.minWidth) * self.AngleField / self.WidthField) + self.minAngle
        return angle


    def writeRequestMsg( self, servo_id, angle, duration ):
        '生成写的消息'
        if angle > 180 or angle < 0 :
            angle = angle % self.AngleField
        self.request()
        self.num_ = servo_id + self.id_start
        self.type_ = 1
        self.data_ = self.angleToWidth(angle) * 10000 + duration


    def readRequestMsg( self, servo_id ):
        '生成读的消息'
        self.request()
        self.num_ = servo_id + self.id_start
        self.type_ = 3
        self.data_ = 0
        

    def readRespondMsg( self, NUM, DATA ):
        '读请求 >> 写入串口 >> 读取串口 >> 解码 >> 调用本函数 >> 读取最后的角度数据'
        '输入解码后的NUM字段和DATA字段，确认舵机id号保存到num_中，返回真实的角度信息'
        self.respond()
        self.num_ = NUM - self.id_start
        self.data_ =DATA
        angle = self.widthToAngle(DATA)
        return angle

    def resetRequestMsg( self, servo_id ):
        '对某个舵机进行复位'
        self.request()
        self.writeRequestMsg(servo_id, (self.minAngle + self.maxAngle)/2, 100 )


class MosSwitch(Equipment):
    '''
    MOS管开关
    '''

    '''
    MOS管开关取值范围为 1111 0101 ~ 1111 0199 共99个。记录开始值。对应设备 1~99号MOS管开关
    '''
    id_start = 11110000

    def __init__(self):
        Equipment.__init__(self)
    
    def close(self, mos_id):
        self.request()
        self.num_ = mos_id + self.id_start
        self.type_ = 1
        self.data_ = 0
    
    def open(self, mos_id):
        self.request()
        self.num_ = mos_id + self.id_start
        self.type_ = 2
        self.data_ = 0

'''
def main():
    s = SerialServo()
    s.writeRequestMsg(3,30,100)
    #s.readRequestMsg(5)
    s.info()
    
if __name__ == '__main__':
    main()
'''