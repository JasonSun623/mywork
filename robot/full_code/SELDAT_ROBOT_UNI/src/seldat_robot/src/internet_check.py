#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Takes messages on server
    Author: Robot Team
      
"""

import rospy
import sys
import time
import math
import numpy
from std_msgs.msg import Int32, String

#import urllib.request #python 3
import urllib

class wifi_check(object):
    
    def __init__(self):
        '''
        rospy.init_node("INTERNET_NODE")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        #self.rate = rospy.get_param('~rate', 0.5)
        '''
        print "Initializing Wifi Status Class..."
        self.wifiMsg = 1001
        #self.rate = rospy.get_param('~rate',1)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',3)
        self.ticks_since_target = 0
        self.wifi_pub = rospy.Publisher('wifi_check',Int32,queue_size = 10)
        rospy.Subscriber("checkAliveTimeOut", String, self.CheckAliveCallback) 
    '''
    def spin(self):
        r = rospy.Rate(self.rate)
        self.ticks_since_target = self.timeout_ticks
        while not rospy.is_shutdown():
            #self.spinOnce()
            self.connection()
            r.sleep()

    def spinOnce(self):
        while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
            #self.ticks_since_target += 1
            self.connection()
            self.ticks_since_target += 1
    '''
    def connection(self):
        if (self.ticks_since_target < self.timeout_ticks):
            #urllib.request.urlopen(host) #python 3
            #urllib.urlopen('https://www.google.com')
            self.wifiMsg = 1000
            self.wifi_pub.publish(self.wifiMsg)
            rospy.loginfo ("Connected to server")
            time.sleep(1)
            #return True
        else:
            self.wifiMsg = 1001
            self.wifi_pub.publish(self.wifiMsg) #1001
            rospy.loginfo ("Server lost connection")
            time.sleep(1)
            #return False
        self.ticks_since_target += 1
    def CheckAliveCallback(self, msg):
        self.ticks_since_target = 0

#test
if __name__=='__main__':
    """ main """
    '''
    wifi = wifi_check()
    wifi.spin()

    '''
    rospy.init_node('Wifi_Node', anonymous=True)
    wifi = wifi_check()
    try:
        while not rospy.is_shutdown():
            wifi.connection()
    except rospy.ROSInterruptException:
            rospy.logwarn("Error in main function")
#print( 'connected' if connected() else 'no internet!' )
