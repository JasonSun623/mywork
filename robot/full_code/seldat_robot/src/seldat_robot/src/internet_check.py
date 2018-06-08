#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy
from std_msgs.msg import Int32

#import urllib.request #python 3
import urllib

class wifi_check(object):
    
    def __init__(self):
        print "Initializing Wifi Status Class..."
        self.wifiMsg = Int32()     
	self.wifi_pub = rospy.Publisher('wifi_check',Int32,queue_size = 10)
	
    def connection(self):
        try:
            #urllib.request.urlopen(host) #python 3
            urllib.urlopen('https://www.google.com')
            self.wifi_pub.publish(1000)
            time.sleep(2)
            #return True
        except:
            self.wifi_pub.publish(1001)
            time.sleep(2)
            #return False

   # def 

#test
if __name__=='__main__':
    """ main """
    rospy.init_node('Wifi_Node', anonymous=True)
    wifi = wifi_check()
    try:
        while not rospy.is_shutdown():
            wifi.connection()
    except rospy.ROSInterruptException:
            rospy.logwarn("Error in main function")
#print( 'connected' if connected() else 'no internet!' )
