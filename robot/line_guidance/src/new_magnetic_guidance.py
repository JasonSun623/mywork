#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May 15 15:39:12 2018

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
import socket


class line_follow():

    ##############################____INIT____############################
    
    def __init__(self):
         ##############################
        self.host = "192.168.1.200"
        self.port = 8081
        ##############################
        self.laser_data = []
        self.PID_enable = 0
        self.lift_val = 3
        self.stop_flag = 0
        self.stop_encoder = 0
        self.flag_1 = 0
        self.stop_flag_laser = 0
        self.flag = 0
        self.flag_laser = 0
        self.mag_ss = []
        self.error = 0
        self.mag_add_flag = 3
        self.set_point = 8
        self.last_proportional = 0
        self.integral = 0
        self.home_value = 5200
        self.angle = 0
        self.temp = 0
        self.pos_left = 0
        self.pos_right = 0
        self.pos_stop = 0
        self.center = 1
        self.list_a = [7,6,5]
        self.list_b = [10,11,12]
        self.stear_enc = 0
        self.t_enc = 0
        self.temp_1 = 0
        self.time = 0
        self.cross_detect = 0
        self.take_pallet = 0
        self.status = 0
        self.temp_2 = 0 
        self.count = 0
        self.count_1 = 0
        self.count_2 = 0
        self.count_3 = 0
        self.count_4 = 0
        self.count_5 = 0
        self.count_6 = 0
        self.count_7 = 0
        self.count_8 = 0
        self.temp_enc = 0
        self.now_encoder = 0
        self.last_encoder = 0
        self.last_encoder_1 = 0
        self.last_encoder_2 = 0
        self.loss_line_flag = 0
        self.loss_line_flag_1 = 0
        self.loss_line_temp = 0
        self.loss_line_stt = 0
        self.count_lane = 0
        self.loss_line_temp_1 = 0
        self.loss_line_temp_2 = 0
        self.loss_line_temp_3 = 0
        self.loss_line_temp_4 = 0
        self.loss_line_temp_5 = 0
        self.loss_line_temp_6 = 0
        self.count_magss = 0
        self.count_3 = 0
        ##########################__INIT_NODE__############################## 
        rospy.init_node('LINE_FOLLOWER')
        
        ###########################__SUBSCRIBER__#############################
        mag_sub = rospy.Subscriber('/magnetline', String, self.Mag_callback)
        mag_sub = rospy.Subscriber('/magnetlineadd', String, self.Mag_add_callback)
        laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_callback)
        lift_sub = rospy.Subscriber('/lift', UInt32, self.lift_callback)
        key_sub = rospy.Subscriber('/key_press', String, self.key_callback)
        stear_enc = rospy.Subscriber('/pos', Vector3, self.stear_callback)
        t_enc = rospy.Subscriber('/pos', Vector3, self.t_callback)
        pallet_pos_sub = rospy.Subscriber('/pospallet',Int32 , self.pallet_pos_callback)
        ###########################__PUBLISH__##############################
        self.vel_pub = rospy.Publisher('twheel',Int32,queue_size = 100)
        self.ste_pub = rospy.Publisher('swheel',Int32,queue_size = 100)
        self.error_pub = rospy.Publisher('errorDetectedLine',Int32,queue_size = 100)
        self.lift_pub = rospy.Publisher('lift_controll', String,queue_size = 100)
        self.program_pub = rospy.Publisher('linedetectioncallback', Int32,queue_size = 100)
        self.line_pos = rospy.Publisher('line_pos',Int32,queue_size = 100)
        
        
    ##############################____CALL_BACK____#########################
            
        #######################__MAGNETIC_SENSOR__########################
    
    def Mag_callback(self,msg):
        mag_sensor = str(msg.data)
        mag_sensor_list = np.array(list(mag_sensor),dtype=np.int)
        mag_value_not = np.logical_not(mag_sensor_list)
        self.mag_ss = map(int,mag_value_not)
        self.count_magss = mag_sensor.count('0')
        if self.count_magss > 9:
            self.cross_detect = 1
        else:
            self.cross_detect = 0
        # REMEMBER CHECK THIS FUNCTION(WRITE NEW 14/11/2017 BY DONGHO)
        #######################__MAGNETIC_ADD_SENSOR__########################
    
    def Mag_add_callback(self,msg):
        mag_sensor = str(msg.data)
        self.mag_add_flag = int(mag_sensor[0])
        #######################__LASER__########################
        
    def laser_callback(self,msg):
        del self.laser_data[:]
        for i in range(0,180):
            self.laser_data.append(msg.ranges[i])
        self.stop_flag_laser = list(filter(lambda x: x > 0 and x < 0.9  ,self.laser_data))
        self.stop_flag_laser = len(self.stop_flag_laser)
        
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
        elif data == 1206:
            self.PID_enable = 4
        elif data == 1207 :
            self.PID_enable = 5
            
        ##########################__LIFT__###########################
    
    def lift_callback(self,msg):
        self.lift_val = msg.data
        
    ##########################__Key_callback__###########################
    
    def key_callback(self,msg):
        data = msg.data
        if data == 'stop pid':
            self.PID_enable = 2
        elif data == 'key p is press' :
            self.PID_enable = 1
        elif data == 'key m is press' :
            self.PID_enable = 2
        elif data == 'key l is press' :
            self.PID_enable = 3
        elif data == 'key j is press' :
            self.PID_enable = 4
        elif data == 'key k is press':
            self.PID_enable = 5
    ##########################__stear_callback__###########################
    def stear_callback(self,msg):
        self.stear_enc = msg.z
        
    ##########################__t_callback__###########################
    def t_callback(self,msg):
        self.t_enc = msg.x
        
    ##########################__POSITION_CALLBACK__###########################
    def pallet_pos_callback(self,msg):
        data = msg.data
        if data == 1205:
            self.pos_stop = 1
        else:
            self.pos_stop = 0

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
        elif pos > 9:
            self.pos_left = 0
            self.pos_right = 1
            self.center = 0
        return pos
    
def mag_flag(self,mag):
        if mag == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            self.count_1 +=1
            if self.count_1 >= 50:
                self.count = 0
                return 1
        else:
            self.count_1 = 0
            return 0
            
        #####################__PID_calculation__#####################
    
    def pid_cal(self,pos,kp,kd):
        proportional = pos - self.set_point
        if self.cross_detect == 1:
            proportional = self.last_proportional
        else:
            proportional = proportional
        #self.integral = self.integral + proportional
        derivative = (proportional - self.last_proportional)
        ster_value = int(proportional* kp +  derivative*kd)# + self.integral*ki) #- 1
        self.last_proportional = proportional
        #print "pos = ",pos,"ster_value = ",ster_value
        time.sleep(0.06)
        return ster_value
       
        #####################__angle_control__#####################
    
    def angle_controll(self,speed):
        pos = self.position(self.mag_ss)
        turning_value = self.pid_cal(self.position(self.mag_ss),23,5)
        #print turning_value,int(turning_value)
        self.angle = self.home_value + turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        if self.angle > 5200:
            self.angle += 75
        if self.angle < 5200:
            self.angle += 50
        else:
            self.angle = self.angle
        print "self.angle",self.angle