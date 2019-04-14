#!/usr/bin/env python 
# -*- coding: utf-8 -*-

'''
不断的瞄准
一旦进入射击距离内，立刻开火。
通过监听火焰坐标 然后进行坐标系变换到pzt坐标系下。然后根据火焰相对于pzt坐标系的坐标，进行不断瞄准，最后在距离达到一定程度时，立刻开火。
'''
import roslib
import rospy
import tf
import math
import time
import cmath
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PointStamped, Vector3
from achilles_store.srv import *



class Aiming(object):
    
    def __init__(self):
        rospy.init_node('exploring_slam', anonymous=True)  
        rospy.on_shutdown(self.shutdown)

        #监听坐标树，以方便使用tf来帮助我们计算机器人坐标变换
        self.tf_listener = tf.TransformListener()

        #如果发现火焰坐标，那么将置为真
        self.emergency = False

        #如果机器人抵达安全边境，那么就停止移动，并打开
        #self.stopDriveToFight = False


        #订阅火焰坐标话题
        self.fire_pose_sub = rospy.Subscriber( "/flame/3D_position", PointStamped, self.firePoseCB )

        #先关闭水泵
        self.peace()

        self.count = 0
        loop_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print("in while %d"%self.count),
            self.count += 1
            self.count = self.count % 1000
            #如果处于紧急状况下,并且导航已经停止
            
            self.emergency = rospy.get_param("isFire",False)
            print(self.emergency)
            
            #and self.isDriving
            if self.emergency:
                if self.count % 2 == 0:
                    self.fighting()
                else:
                    pass
            else:
                if self.count % 2 == 0:
                    self.peace()
                else:
                    pass
            loop_rate.sleep()
        rospy.spin()
        

    def fighting(self):
        '作战'
        print("fighting")
        t_x = self.fire_point.point.x
        t_y = self.fire_point.point.y
        t_z = self.fire_point.point.z
        self.safeZone = rospy.get_param("safeZone",1.5)
        #self.stopDriveToFight = rospy.get_param("stopDriveToFight",False)

        #设置安全区
        if t_z < self.safeZone :
            angle_z = math.atan2(t_y,t_x)
            #如果高于云台，那么将会绕y轴进行反方向运动
            angle_y = - math.atan2(
                t_z,
                pow(pow(t_x,2) + pow(t_y,2), 0.5)
            )
            #print("绕Z轴旋转 %f "%angle_z),
            #print("绕Y轴旋转 %f "%angle_y)

            if angle_z > 0:
                angle_z = (angle_z + 0.1745)* 1.15
            else:
                angle_z = angle_z * 1.15

            rospy.wait_for_service("/pzt/pose_cmd")
            rospy.wait_for_service("/pzt/switch")
            try:
                print("服务开始执行")
                pose_cmd = rospy.ServiceProxy('/pzt/pose_cmd', poseDestination)
                switch = rospy.ServiceProxy('/pzt/switch', mosSwitch)
                #此处使用的是委托服务
                pose_cmd_res_1 = pose_cmd(1, Vector3(0,0,angle_z))
                time.sleep(0.5)
                pose_cmd_res_2 = pose_cmd(2, Vector3(0,angle_y*0.8,0))
                time.sleep(0.5)
                #打开水泵
                switch_state = switch(1,True)
                time.sleep(0.5)
                print("服务执行完毕")
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
        else:
            print ("out of fighting range 目标超出射程")
    

    def peace(self):
        '休战'
        print("peacing")
        rospy.wait_for_service("/pzt/switch")
        try:
            print("服务开始执行")
            switch = rospy.ServiceProxy('/pzt/switch', mosSwitch)
            #关闭水泵
            switch_state = switch( 1, False)
            print("服务执行完毕")
            #如果打开是假，即水泵成功关闭，那么将停止移动设置为假
            '''
            if not switch_state:
                rospy.set_param("stopDriveToFight",False)
            '''
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



    def firePoseCB(self, data):
        print("firePoseCB"),
        print(self.count)
        try:
            #告诉tf变换 ，我需要的是这个点在"pzt_link"下的坐标，data这个点是在摄像头坐标系下的获得的，self.fire_point"base_link"下的信息
            self.fire_point = self.tf_listener.transformPoint("/pzt_link",data)
        except Exception,e:
            print (e)
            rospy.logerr('firePoseCB error')

    def shutdown(self):
        self.peace()

def main():
    aiming = Aiming()


if __name__ == '__main__':
    main()