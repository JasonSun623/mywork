#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
import time
import math
import numpy
from math import sin, cos, pi
from std_msgs.msg import Float32,UInt16,Int16,UInt8,Int8,Int32, UInt32,Int64,String,UInt8MultiArray,Float32MultiArray

from geometry_msgs.msg import Vector3, Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class Arduino_Motor_class(object):

	def __init__(self):
		print "Initializing Line To CMD Class..."
		self.velMsg = Twist()
		self.vel_robot = 0
		self.rad_robot = 0
		self.r_robot = 0
		rospy.Subscriber("motor_command",Vector3, self.motorCallback)
		self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi",Twist,queue_size = 10)
		self.line_stop_rad_pub = rospy.Publisher("line_stop_rad",Float32,queue_size = 10)
        def motorCallback(self, msg):
		t_vel = (msg.x * 0.2)/1000
		r_vel = (msg.z - 2047)/1296
		self.r_robot = 0

		if (t_vel == 0):
			self.r_robot = r_vel

		if (r_vel > 1.57):
			r_vel = 1.57
		elif (r_vel < -1.57):
			r_vel = -1.57

		#print (msg.x)
		#print (msg.z)
		self.vel_robot = t_vel * cos(r_vel)
		self.rad_robot = sin(r_vel)*(t_vel/1.745)

		self.velMsg.linear.x = self.vel_robot
		self.velMsg.angular.z = self.rad_robot

		self.vel_pub.publish(self.velMsg)
		self.line_stop_rad_pub.publish(self.r_robot)
        #time.sleep(0.001)
	
if __name__ == '__main__':
    	rospy.init_node('arduino_control_motor', anonymous=True)
	#rospy.Rate(10) # 10hz
	arduino = Arduino_Motor_class()
	rospy.spin()
