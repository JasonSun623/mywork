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
        print "Initializing Info Display Class..."
	#self.x_data = 0
	#self.z_data = 0
	self.manual_data = "\"no\""
	self.wifi_data = "\"no\""
	self.chargeCT = "\"no\""
	self.battery_data = 0.0
	self.vel = 0.0
	self.odom = 0.0
        
	self.infoMsg = String()
        self.info_pub = rospy.Publisher('robotInfoDisplay',String,queue_size = 10)

	#rospy.Subscriber("pos", Vector3, self.posCallback)
	rospy.Subscriber("odom", Odometry, self.odomCallback)
	rospy.Subscriber("manual", Int32, self.manualCallback)
	rospy.Subscriber("wifi_check", Int32, self.wifiCallback)
	rospy.Subscriber("battery_vol", Float32, self.batteryCallback)
	rospy.Subscriber("chargeCT", Int32, self.chargeCallback)
	
    def info(self):
	self.infoMsg = "{\"robot\": \"Robot_3rd\", \"speed\":" + str(self.odom) + ",\"wifi\":" + self.wifi_data + ",\"charge\":" + self.chargeCT +", \"manual\":" + self.manual_data + ",\"battery\":" +str(self.battery_data) +"}"
	self.info_pub.publish(self.infoMsg)
        time.sleep(0.5)
	
    #def posCallback(self, msg):
    def odomCallback(self, msg):
	self.odom = msg.twist.twist.linear.x * 3.6
	
    def manualCallback(self, msg):
	if (msg.data == 1002):
		self.manual_data = "\"yes\""
	elif (msg.data == 1802):
		self.manual_data = "\"no\""

    def wifiCallback(self, msg):
	if (msg.data == 1000):
		self.wifi_data = "\"yes\""
	elif (msg.data == 1001):
		self.wifi_data = "\"no\""
	
    def batteryCallback(self, msg):
	self.battery_data = msg.data

    def chargeCallback(self, msg):
	if (msg.data == 5101):
		self.chargeCT = "\"yes\""
	elif (msg.data == 5100):
		self.chargeCT = "\"no\""
		
#test
if __name__=='__main__':
    """ main """
    rospy.init_node('Info_Node', anonymous=True)
    info = info_display()
    try:
        while not rospy.is_shutdown():
            info.info()
    except rospy.ROSInterruptException:
            rospy.logwarn("Error in main function")
#print( 'connected' if connected() else 'no internet!' )
