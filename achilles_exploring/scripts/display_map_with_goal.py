#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetMapRequest,GetMapResponse
from geometry_msgs.msg import PoseStamped



class Map(object):
    '''
    地图类
    '''
    _delta = rospy.get_param("/slam_gmapping/delta", 0.05)

    def __init__(self):
        '''
        rospy订阅map话题，第二个是数据类型，第三个是回调函数
        将订阅的数据传给回调函数，就是那个mapmsg变量
        如果有话题来了，就直接调用callback函数
        '''
        
        print ("delta : %s"%self._delta)
        self.map_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback )

        #self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.callback)

        print ("get map~")
        
        

    def call(self):
        rospy.wait_for_service('/dynamic_map')
        try:
            mapClient = rospy.ServiceProxy('/dynamic_map', GetMap)
            #此处使用的是委托服务
            res = mapClient()
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def callback(self, msg):
        '''
        回调函数
        '''
        try:
            
            position =  [int(msg.pose.position.x / self._delta),
                         int(msg.pose.position.y / self._delta), 
                         int(msg.pose.position.z / self._delta)]
            print ( "Real Goal : %s"%position)
            
            res = self.call()
            mapArray = np.array(res.map.data)
            cols = res.map.info.width
            rows = res.map.info.height
            mapArray =  mapArray.reshape((rows, cols))
            #print mapArray
            #print mapArray.shape
            #print type(mapArray)
            map = np.empty((rows,cols,3), dtype = np.uint8)
            #BGR
            map.fill(127)
            #数组中 未知区域的值为  -1，标记为灰色(默认色[127, 127, 127])
            #      无障碍区域的值为  0，标记为白色([255, 255, 255])
            #      有障碍区域的值为  100，标记为黑色([0, 0, 0])
            grey = np.array([127, 127, 127])
            whilt = np.array([255, 255, 255])
            black = np.array([0, 0, 0])
            red = np.array([0, 0, 255])
            
        except Exception,e:
            print (e)
            rospy.logerr('convert rgb image error')

        for i in range(0, rows):
            for j in range(0, cols):
                if (mapArray[i,j] == 100):
                    map[rows-i-1,j] = black
                if (mapArray[i,j] == 0):
                    map[rows-i-1,j] = whilt
        
        #map[position[0] + rows / 2, position[1] + cols / 2] = red
        #金色目标点标记
        cv2.circle(map, (position[0] + cols / 2 + 1, rows /2 - 1 - position[1]), 4, (0, 215, 255),-1)
        s = time.strftime("maps/%Y.%m.%d-%H:%M:%S.jpg", time.localtime(time.time()))
        print ("Saving...  "),
        cv2.imwrite(s, map)
        print ("Saved")


if __name__=='__main__':
    rospy.init_node('display_map_with_goal', anonymous=False)
    v = Map()
    rospy.spin()