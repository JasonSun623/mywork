#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import time
from std_msgs.msg import UInt16,UInt8,Int8,Int32, UInt32, String,UInt8MultiArray,Float32
from geometry_msgs.msg import Quaternion,Vector3
from math import pi
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

class visual_data():
    def __init__(self):
        self.delay = 1
        rospy.init_node('visal_data')
        self.lift_pub = rospy.Publisher('/magnetline', String,queue_size = 100)
        self.list = ["11111111001111111","11111110011111111","11111100111111111","11111001111111111","11110011111111111","11100111111111111","11001111111111111","10011111111111111","10111111111111111"]
        self.list1 = ["11111111001111111","11111111100111111","11111111110011111","11111111111001111","11111111111100111","11111111111100111","11111111111110011","11111111111111001","11111111111111100","11111111111111110"]
    def main(self):
        rospy.Rate(20)
        while not rospy.is_shutdown():
            for i in self.list :
                print i
                self.lift_pub.publish(i)
                time.sleep(self.delay)
            for i in self.list[::-1] :
                print i
                self.lift_pub.publish(i)
                time.sleep(self.delay)
            for i in self.list1 :
                print i
                self.lift_pub.publish(i)
                time.sleep(self.delay)
            for i in self.list1[::-1] :
                print i
                self.lift_pub.publish(i)
                time.sleep(self.delay)
if __name__ == '__main__':
    run = visual_data()
    run.main()
