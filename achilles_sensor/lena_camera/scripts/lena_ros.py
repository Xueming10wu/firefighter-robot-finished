#!/usr/bin/env python 
# -*- coding: utf-8 -*-

from __future__ import print_function
import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


        
class image_converter:
    def __init__(self):
        rospy.init_node('lena_node', anonymous=False)
        self.dev_port = rospy.get_param("/lena_node/Lena_dev", 1)
        print("dev %d"%self.dev_port)
        self.__setup()
        self.L_image_pub = rospy.Publisher("/Lena/left/image_raw", Image, queue_size = 1)
        self.R_image_pub = rospy.Publisher("/Lena/right/image_raw", Image , queue_size = 1)

        self.L_bridge = CvBridge()
        self.R_bridge = CvBridge()
        self.publishImage()

    def __setup(self):
        #设置 读取 以及 读取尺寸
        self.capture = cv2.VideoCapture(self.dev_port)
        #self.capture.open(1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.width, self.height = self.capture.get(3), self.capture.get(4)
        self.loop_rate = rospy.Rate(30)
    
    def publishImage(self):
        print("start")
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if ret:
                frames = np.split(frame, 2, axis = 1)
                frame_R = frames[0]
                frame_L = frames[1]
                #cv2.imshow("frame_R", frame_R)
                #cv2.imshow("frame_L", frame_L)

                r_msg = self.R_bridge.cv2_to_imgmsg(frame_R, "bgr8")
                l_msg = self.L_bridge.cv2_to_imgmsg(frame_L, "bgr8")

                t = rospy.Time.now()
                r_msg.header.stamp = t
                l_msg.header.stamp = t
                r_msg.header.frame_id = "left_camera_link"
                l_msg.header.frame_id = "right_camera_link"


                r_msg.width = self.width
                l_msg.width = self.width
                r_msg.height = self.height
                l_msg.height = self.height

                try:
                    self.L_image_pub.publish(l_msg)
                    self.R_image_pub.publish(r_msg)
                except CvBridgeError as e:
                    print(e)

                self.loop_rate.sleep()
            else:
                print("\033[1;31;47mno this device : %d\033[0m"%self.dev_port)
                break


def main():
    ic = image_converter()
    rospy.spin()

if __name__ == '__main__':
    main()