#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 08:46:12 2017

@author: dongho
"""


import rospy
import time
from std_msgs.msg import UInt16,UInt8,Int8,Int32, UInt32, String,UInt8MultiArray,Float32
from geometry_msgs.msg import Quaternion,Vector3
from math import pi
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

class line_follow():
    
    ##############################____INIT____############################
    
    def __init__(self):
        self.laser_data = []
        self.PID_enable = 0
        self.lift_val = 3
        self.stop_flag = 0
        self.flag = 0
        self.mag_ss = []
        self.error = 0
        self.set_point = 8
        self.set_point_1 = 9
        self.last_proportional = 0
        self.integral = 0
        self.home_value = 54
        self.angle = 0
        self.temp = 0
        self.pos_left = 0
        self.pos_right = 0
        self.center = 1
        self.list_a = [7,6,5]
        self.list_b = [10,11,12]
        self.stear_enc = 0
        self.t_enc = 0
        self.temp_1 = 0
        ###########################__INIT_NODE__############################## 
        rospy.init_node('LINE_FOLLOWER')
        
        ###########################__SUBSCRIBER__#############################
        mag_sub = rospy.Subscriber('/magnetline', String, self.Mag_callback)
        laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_callback)
        lift_sub = rospy.Subscriber('/lift', UInt32, self.lift_callback)
        key_sub = rospy.Subscriber('/key_press', String, self.key_callback)
        stear_enc = rospy.Subscriber('/pos', Vector3, self.stear_callback)
        t_enc = rospy.Subscriber('/pos', Vector3, self.t_callback)
        ###########################__PUBLISH__##############################
        self.vel_pub = rospy.Publisher('twheel',Int32,queue_size = 100)
        self.ste_pub = rospy.Publisher('swheel',Int32,queue_size = 100)
        self.lift_pub = rospy.Publisher('lift_controll', String,queue_size = 100)
        self.program_pub = rospy.Publisher('linedetectioncallback', Int32,queue_size = 100)
        
    ##############################____CALL_BACK____#########################
            
        #######################__MAGNETIC_SENSOR__########################
    
    def Mag_callback(self,msg):
        mag_sensor = str(msg.data)
        mag_sensor_list = np.array(list(mag_sensor),dtype=np.int)
        mag_value_not = np.logical_not(mag_sensor_list)
        self.mag_ss = map(int,mag_value_not)
        # REMEMBER CHECK THIS FUNCTION(WRITE NEW 14/11/2017 BY DONGHO)
        
        #######################__LASER__########################
        
    def laser_callback(self,msg):
        del self.laser_data[:]
        for i in range(0,180):
            self.laser_data.append(msg.ranges[i])
        self.stop_flag = list(filter(lambda x: x > 0 and x < 0.9  ,self.laser_data))
        self.stop_flag = len(self.stop_flag)
        
        ##########################__SERVER__#########################
    def server_callback(self,msg):                      
        data = msg.data
        if data == 1203:
            self.PID_enable = 1
        elif data == 1200:
            self.PID_enable = 2
            self.status = 0
        elif data == 1204 :
            self.PID_enable = 3
            
        ##########################__LIFT__###########################
    
    def lift_callback(self,msg):
        self.lift_val = msg.data
        
    ##########################__Key_callback__###########################
    
    def key_callback(self,msg):
        data = msg.data
        if data == 'stop pid':
            self.PID_enable = 0
        elif data == 'key p is press' :
            self.PID_enable = 1
        elif data == 'key m is press' :
            self.PID_enable = 2
        elif data == 'key l is press' :
            self.PID_enable = 3
    ##########################__stear_callback__###########################
    def stear_callback(self,msg):
        self.stear_enc = msg.z
        
    ##########################__t_callback__###########################
    def t_callback(self,msg):
        self.t_enc = msg.x
    #############################____FUNCTIONs____##########################
        
        ####################__error_calculation__#####################
        
    def position(self,mag_ss):
        ss_sum = 0
        ss_aveg = 0
        for i in range(0,len(mag_ss)) :
            ss_aveg += mag_ss[i] * i
            ss_sum += int(self.mag_ss[i])
        if ss_sum == 0:
            pos = 8
        else:
            pos = (ss_aveg/ss_sum) + 1
        if pos < 7:
            self.pos_left = 1
            self.pos_right = 0
            self.center = 0
        elif pos > 10:
            self.pos_left = 0
            self.pos_right = 1
            self.center = 0
        print "pos = ",pos
        return pos
            
        #####################__PID_calculation__#####################
    
    def pid_cal(self,pos,kp,kd):
        proportional = pos - self.set_point
        #self.integral = self.integral + proportional
        derivative = (proportional - self.last_proportional)
        ster_value = int(proportional* kp +  derivative*kd) - 1
        self.last_proportional = proportional
        #print "pos = ",pos,"ster_value = ",ster_value
        return ster_value
        
    #####################__angle_control__#####################
    def angle_controll(self,speed):
        turning_value = self.pid_cal(self.position(self.mag_ss),4,12)
        #print turning_value,int(turning_value)
        self.angle = self.home_value + turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        #print "self.angle",self.angle
        if self.angle > 100:
            self.angle = 100
        elif self.angle < 5:
            self.angle = 5
        elif self.angle >5 and self.angle < 35:
            self.angle = 35
        
        print "speed",speed, "angle",self.angle
        #print "left = ", self.left,"right = ",self.right,"center = ",self.center
        self.vel_pub.publish(speed)
        self.ste_pub.publish(self.angle)
        #####################__moving_to_pallet__#####################
    def moving_to_pallet(self):
        pos = self.position(self.mag_ss)
        if self.stop_flag == 1:
            self.angle_controll(-1000)
        elif self.stop_flag == 0 and self.pos_left == 1:
            print " in the left"
            if pos == 8:
                self.temp_1 = 1
                self.vel_pub.publish(-50)
                self.ste_pub.publish(54)
            elif self.temp_1 == 1 and self.stop_flag == 0:
                self.temp = self.temp - 1
                self.vel_pub.publish(-500)
                self.ste_pub.publish(85)
                if self.temp <= 10:
                    self.stop_flag = 1
                    self.temp = 0
                    self.temp_1 = 0
            else:
                self.temp = self.temp + 1
                self.angle_controll(-500)
        elif self.pos_right == 1 and self.stop_flag == 0:
            print "in the right"
            if pos == 8:
                self.vel_pub.publish(-50)
                self.ste_pub.publish(54)
                self.temp_1 = 1
            elif self.temp_1 == 1 and self.stop_flag == 0:
                self.temp = self.temp - 1
                self.vel_pub.publish(-500)
                self.ste_pub.publish(63)
                if self.temp <= 10:
                    self.stop_flag = 1
                    self.temp = 0
                    self.temp_1 = 0
            else:
                self.temp = self.temp + 1
                self.angle_controll(-500)
        else:
	    self.stop_flag = 1
	    selft.temp_1 = 0
            #self.angle_controll(-800)
    ################################__MAIN__###################################
    def main(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.PID_enable == 0:
                self.vel_pub.publish(0)
                self.ste_pub.publish(54)
                self.laser_data = []
                self.PID_enable = 0
                self.lift_val = 3
                self.stop_flag = 0
                self.flag = 0
                self.mag_ss = []
                self.error = 0
                self.set_point = 8
                self.set_point_1 = 9
                self.last_proportional = 0
                self.integral = 0
                self.home_value = 54
                self.angle = 0
                self.temp = 0
                self.pos_left = 0
                self.pos_right = 0
                self.center = 1
                self.list_a = [7,6,5]
                self.list_b = [10,11,12]
                self.stear_enc = 0
                self.t_enc = 0
                self.temp_1 = 0
            elif self.PID_enable == 1:
                self.moving_to_pallet()
            #print self.stear_enc,self.t_enc
            #print self.position(self.mag_ss)
            r.sleep()
        	#print "self.mag_ss",self.mag_ss
		#print "self.stop_flag",self.stop_flag
		#print "self.lift_val",self.lift_val
if __name__ == '__main__':
    run = line_follow()
    run.main()
