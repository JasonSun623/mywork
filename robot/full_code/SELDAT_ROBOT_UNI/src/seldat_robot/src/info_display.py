#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry


#import urllib.request #python 3
import urllib

class info_display(object):
    
    def __init__(self):
	
        rospy.init_node("INFO_NODE")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = rospy.get_param('~rate', 2)
	
        print "Initializing Info Display Class..."
        #self.x_data = 0
        #self.z_data = 0
        self.manual_data = "\"no\""
        self.chargeCT = "\"no\""
        self.pc_error = 1000
        self.pc_warning = 2000
        self.stm_error = 1000
        self.stm_warning = 2000
        self.battery_data = 24.0
        self.vel = 0.0
        self.odom = 0.0
        
        self.infoMsg = String()
        self.info_pub = rospy.Publisher('robotInfoDisplay',String,queue_size = 10)

        #rospy.Subscriber("pos", Vector3, self.posCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        rospy.Subscriber("manual", Int32, self.manualCallback)
        rospy.Subscriber("wifi_check", Int32, self.wifiCallback)
        rospy.Subscriber("battery_vol", Float32, self.batteryCallback)
        #rospy.Subscriber("chargeCT", Int32, self.chargeCallback)
        rospy.Subscriber("stop_flag", Int32, self.obstaclesCallback)
        rospy.Subscriber("stm_error", Int32, self.stm_errorCallback)
        rospy.Subscriber("stm_warning", Int32, self.stm_warningCallback)
	
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.info()
            r.sleep()

    def info(self):
        self.infoMsg = "{\"robot\": \"Robot_2\", \"speed\":" + str(self.odom) + \
                       ",\"manual\":" + self.manual_data + ",\"battery\":" +str(self.battery_data) + \
                       ",\"pc_error\":" +str(self.pc_error) + ",\"pc_warning\":" +str(self.pc_warning) + \
                       ",\"stm_error\":" +str(self.stm_error) + ",\"stm_warning\":" +str(self.stm_warning)+"}"
        self.info_pub.publish(self.infoMsg)
        #time.sleep(0.5)
	
    def odomCallback(self, msg):
        self.odom = msg.twist.twist.linear.x * 3.6
	
    def manualCallback(self, msg):
        if (msg.data == 1):
            self.manual_data = "\"yes\""
        elif (msg.data == 0):
            self.manual_data = "\"no\""

    def wifiCallback(self, msg):
        self.pc_error = msg.data
	
    def batteryCallback(self, msg):
        self.battery_data = msg.data
    '''
    def chargeCallback(self, msg):
        if (msg.data == 5101):
            self.chargeCT = "\"yes\""
        elif (msg.data == 5100):
            self.chargeCT = "\"no\""
    '''			
    def obstaclesCallback(self, msg):
        if (msg.data == 1002):
            self.pc_warning = 2001
        else:
            self.pc_warning = 2000
			
    def stm_errorCallback(self, msg):
        self.stm_error = msg.data
			
    def stm_warningCallback(self, msg):
        self.stm_warning = msg.data
		
#test
if __name__=='__main__':
    """ main """
    info = info_display()
    info.spin()
    '''
    rospy.init_node('Info_Node', anonymous=True)
    info = info_display()
    try:
        while not rospy.is_shutdown():
            info.info()
    except rospy.ROSInterruptException:
            rospy.logwarn("Error in main function")
    '''
#print( 'connected' if connected() else 'no internet!' )
