#!/usr/bin/env python         
# -*- coding: utf-8 -*-
from Serial import Serial
from MultCode import MultCode
from Equipment import Equipment,SerialServo,MosSwitch
import math
import time
import rospy
from achilles_store.msg import pose
from achilles_store.srv import poseDestination, poseDestinationRequest, poseDestinationResponse
from achilles_store.srv import mosSwitch,mosSwitchRequest,mosSwitchResponse
from geometry_msgs.msg import Vector3


class ArduinoNode(object):
    def __init__(self):
        '协议与串口加载'
        self.multcode = MultCode()
        self.servo = SerialServo()
        self.mos = MosSwitch()
        '''
        arduino设备规则
        idVendor           0x2341 Arduino SA
        idProduct          0x003d 
        '''
        self.serial = Serial("/dev/arduino_due", 9600)

        'ROS'
        '发布舵机角度的消息'
        self.msg = pose()
        '两个舵机'
        self.msg.ID.append( 1 )
        self.msg.ID.append( 2 )
        self.msg.angular.append(Vector3())
        self.msg.angular.append(Vector3())
        self.msg.header.frame_id = "pzt_link"
        '一个发布者，两个服务端'
        self.pub = rospy.Publisher("/pzt/pose", pose, queue_size=10)
        self.pose_cmd_server = rospy.Service("/pzt/pose_cmd",poseDestination, self.destinationSF )
        self.mos_cmd_server = rospy.Service("/pzt/switch", mosSwitch, self.switchSF )
        '频率设置为2hz'
        self.loop_rate = rospy.Rate(2)
        'IO控制标志位'
        self.IO_bussy = False
        print("arduino 构造完成")
    

    def destinationSF(self,req):
        '写角度使用的服务器'
        #请求参数为弧度角
        PI = 3.1415926
        #弧度角
        degree = PI/180
        
        res = poseDestinationResponse()
        res.angular.x = 0
        res.angular.y = 0
        res.angular.z = 0
        id = req.ID
        if (id == 1):
            self.servo.writeRequestMsg(1, (req.angular.z/degree)  + 90, 100)
            res.angular.z = req.angular.z
        elif (id == 2):
            self.servo.writeRequestMsg(2, (req.angular.y/degree) + 90, 100 )
            res.angular.y = req.angular.y
        else:
            print ("No this servo id")
        '编码'
        s = self.multcode.encode( self.servo.from_, self.servo.to_, self.servo.num_, self.servo.type_, self.servo.data_ )
        print("将向arduino 发送%s"%s)
        while( self.IO_bussy ):
            time.sleep(0.01)
        self.IO_bussy = True
        try:
            self.serial.write(s)
        except Exception as e:
            print ("arduino 写角度服务失败")
            res.angular.x = 0
            res.angular.y = 0
            res.angular.z = 0

        self.IO_bussy = False
        print("arduino 写角度服务完成")
        return res

    
    def switchSF(self, req):
        '写开关使用的服务器'
        res = mosSwitchResponse()
        res.isOpen = req.needOpen
        id = req.ID
        if (id == 1):
            if( req.needOpen ):
                self.mos.open(id)
            else:
                self.mos.close(id)
            '编码'
        else:
            print ("No this mos id")
        s = self.multcode.encode( self.mos.from_, self.mos.to_, self.mos.num_, self.mos.type_, self.mos.data_ )
        while( self.IO_bussy ):
            time.sleep(0.01)
        self.IO_bussy = True

        try:
            self.serial.write(s)
            rx = self.serial.read()
            print ("mos 管发送请求%s"%s)
        except Exception as e:
            print("arduino 开关服务失败")
            res.isOpen  = not req.needOpen
        
        self.IO_bussy = False
        print("arduino 开关服务完成")
        return res
    
    def talker(self):
        '总发布者，实时的将单片机的数据传送回来'
        id_in_one = 1
        publish_permission = [False,False]
        PI = 3.1415926
        degree = PI / 180

        while not rospy.is_shutdown():
            '此循环是用于 发送角度数据的 发布者'
            '选择其中一个舵机号'

            self.servo.readRequestMsg(id_in_one)
            id_in_one = id_in_one % 2 + 1
            tx = self.multcode.encode(self.servo.from_, self.servo.to_, self.servo.num_, self.servo.type_, self.servo.data_)
            #print(tx)

            while( self.IO_bussy ):
                time.sleep(0.01)
            self.IO_bussy = True
            try:
                self.serial.write(tx)
                rx = self.serial.read()
                print(rx)
            except Exception as e:
                print ("写请求失败")
            self.IO_bussy = False

            if( len(rx) == self.multcode.getSumSize() + 1 ):
                self.multcode.decode(rx)
            else:
                print ("数据残缺，丢弃。")
            
            id = self.multcode.getNum() % 100
            #print("id"),
            #print(id)

            readAngle =  (self.servo.readRespondMsg( self.multcode.getNum(), self.multcode.getData()) - 90)
            #print("angle"),
            #print(readAngle)
            if id == 1:
                print("1 %s"%readAngle)
                self.msg.angular[0].z = readAngle * degree
                #publish_permission[0] = True
            elif id == 2:
                print("2 %s"%readAngle)
                self.msg.angular[1].y = readAngle * degree
                #publish_permission[1] = True
            else:
                print("wrong id")
            '''
            if publish_permission[0] and publish_permission[1] :
                print("Publishing\n\n")
                publish_permission[0] = False
                publish_permission[1] = False
                self.pub.publish(self.msg)
            '''
            #self.msg.header.stamp = rospy.Time.now()
            print("Publishing\n\n")
            self.pub.publish(self.msg)
            self.loop_rate.sleep()
            
        self.serial.closePort()


def main():
    rospy.init_node("arduino_node", anonymous = False)
    
    try:
        handle = ArduinoNode()
        handle.talker()
        
    except rospy.ROSInterruptException:
        print ("构造失败")

if __name__ == '__main__':
    main()