#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib
import tf
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PointStamped,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
import math
import time

class NavTest():  
    def __init__(self):  
        
        rospy.on_shutdown(self.shutdown)

        #监听坐标树，以方便使用tf来帮助我们计算机器人坐标变换
        self.tf_listener = tf.TransformListener()

        #如果发现火焰坐标，那么将置为真
        self.emergency = False
        self.isFinished = True

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)
        
        # 设置目标点的位置  
        # 在rviz中点击 2D Nav Goal 按键，然后单击地图中一点  
        # 在终端中就会看到该点的坐标信息  
        locations = []

        
        locations.append(Pose(Point(2.118, -0.313, 0.000), Quaternion(0.000, 0.000, -0.018, 1.000)))
        locations.append(Pose(Point(2.140, 0.015, 0.000), Quaternion(0.000, 0.000, -0.352, 0.936)))
        locations.append(Pose(Point(1.165, 0.829, 0.000), Quaternion(0.000, 0.000, -0.412, 0.911)))
        locations.append(Pose(Point(2.181, -0.068, 0.000), Quaternion(0.000, 0.000, -0.687, 0.726)))
        locations.append(Pose(Point(2.528, -0.057, 0.000), Quaternion(0.000, 0.000, -0.007, 1.000)))
        locations.append(Pose(Point(2.895, -0.082, 0.000), Quaternion(0.000, 0.000, -0.019, 1.000)))
        locations.append(Pose(Point(-1.586, -0.160, 0.000), Quaternion(0.000, 0.000, 0.994, -0.108)))
        locations.append(Pose(Point(3.732, -0.026, 0.000), Quaternion(0.000, 0.000, 0.728, 0.686)))
        locations.append(Pose(Point(2.601, -0.262, 0.000), Quaternion(0.000, 0.000, 0.664, 0.748)))
        locations.append(Pose(Point(4.779, -1.212, 0.000), Quaternion(0.000, 0.000, 0.991, 0.137)))
        '''
        locations.append(Pose(Point(3.329, 0.164, 0.000), Quaternion(0.000, 0.000, -0.697, 0.717)))
        locations.append(Pose(Point(2.099, 0.047, 0.000), Quaternion(0.000, 0.000, -0.713, 0.701)))
        locations.append(Pose(Point(1.985, 0.159, 0.000), Quaternion(0.000, 0.000, -0.066, 0.998)))
        locations.append(Pose(Point(2.035, 0.034, 0.000), Quaternion(0.000, 0.000, 1.000, 0.012)))
        locations.append(Pose(Point(3.485, -0.060, 0.000), Quaternion(0.000, 0.000, -0.733, 0.680)))
        locations.append(Pose(Point(6.088, 0.337, 0.000), Quaternion(0.000, 0.000, 1.000, 0.005)))
        '''
        
        '''
        for i in range(0,len(locations)):
            print (locations[i])
            print ("")
        '''
        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmdCB)

        # 发布导航点
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        #订阅火焰坐标话题
        self.fire_pose_sub = rospy.Subscriber( "/flame/3D_position", PointStamped, self.firePoseCB )


        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  


        #目标点索引
        location_index= 0

        self.current_location = locations[location_index]
        self.next_location = locations[location_index + 1]

        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)

        rospy.loginfo("Starting navigation test")

        #设置目标
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'base_link'

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():
            print("In while")
            self.goal.header.stamp = rospy.Time.now()

            #设定距离火焰的安全距离 单位 m
            self.safeZone = rospy.get_param("safeZone",1.5)

            #获得火情参数
            self.emergency = rospy.get_param("isFire", False)

            #有火情
            if self.emergency:
                print("有火情")
                #计算火焰与目前所在位置的距离
                self.base_distance = sqrt(pow(self.fire_base_point.point.x)  + pow(self.fire_base_point.point.y))
                
                #当前距离已经小于安全距离
                if self.safeZone > self.base_distance:
                    #停止运动，直到某个条件解除，并且设置停止移动的参数
                    rospy.set_param("stopDriveToFight",True)
                    self.stopDriveToFight = True
                    self.shutdown()
                
                #当前距离还没有小于安全距离
                else:
                    #计算下一个目标点坐标
                    #设置在/map坐标系下的下一个目标点
                    #发现火焰的情况下设置一个向量
                    v = [self.fire_map_point.point.x - self.current_location.position.x, self.fire_map_point.point.y - self.current_location.position.y, 0]
                     
                    #tf.transformations.quaternion_from_euler计算出到达目标点时候的目标姿态,使用弧度角获得的欧拉角(z y x)，算出四元素
                    self.next_location = Pose(
                        Point(
                            (self.fire_map_point.point.x + self.current_location.position.x) / 2,
                            (self.fire_map_point.point.y + self.current_location.position.y) / 2,
                            0
                        ), 
                        tf.transformations.quaternion_from_euler(atan( v[1] / v[0] ), 0, 0)
                    )
  
            #没有火情
            else:
                print("没有火情")
                if location_index >= len(locations) :
                    location_index = 0
                #目标点进行更新，索引要在成功之后才能进行变化
                if location_index + 1 >= len(locations):
                    #防止索引越界
                    self.next_location = locations[0]
                else:
                    self.next_location = locations[location_index + 1]
            
            print("放入目标消息中")
            # 放入目标消息中
            self.goal.pose = self.next_location

            while True:
                if self.isFinished:
                    time.sleep(2)
                    if self.isFinished:
                        break
                    else:
                        time.sleep(2)
                else:
                    time.sleep(2)
                
            #等待水泵关闭后
            self.stopDriveToFight = rospy.get_param("stopDriveToFight",True)
            while self.stopDriveToFight:
                print("等待水泵关闭。。。")
                #如果 在停止状态中，那么就休眠
                self.stopDriveToFight = rospy.get_param("stopDriveToFight",True)
                rospy.sleep(self.rest_time)
            
            # 向下一个位置进发  
            self.goal_pub.publish(self.goal)
            print("消息已经发送")
            
            # 查看是否成功到达
            print("等待时间结束")
            location_index = (location_index + 1) % len(locations)
            print("location_index :%d"%location_index)
            # 存储当前所在的位置
            self.current_location = self.next_location
            rospy.sleep(self.rest_time)
          
  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)

    #发现火焰后执行的回调函数
    def firePoseCB(self, data):
        try:
            #告诉tf变换 ，我需要的是这个点在"base_link"下的坐标，data这个点是在摄像头坐标系下的获得的，self.fire_base_point.point保存这个点在"base_link"下的信息
            self.fire_base_point = self.tf_listener.transformPoint("/pzt_link",data)

            #获得火焰在/map坐标系下的坐标关系
            self.fire_map_point = self.tf_listener.transformPoint("/map", data)
        except Exception,e:
            print (e)
            rospy.logerr('firePoseCB error')

    def cmdCB(self, data):
        l = data.linear.x * data.linear.x
        a = data.angular.z * data.angular.z
        if l + a == 0:
            self.isFinished = True
        else:
            self.isFinished = False
        

if __name__ == '__main__':  
    try:  
        rospy.init_node('exploring_slam', anonymous=False)  
        NavTest()
        rospy.spin()
        

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
