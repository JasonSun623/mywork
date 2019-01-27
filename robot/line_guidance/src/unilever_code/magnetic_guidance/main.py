#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 08:10:05 2018

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
import ast

##########################__start_Library_##################################

import config
import read_line_pos as pos
import pid_


##########################__End_Library_##################################

class line_follow():
    
    def __init__(self):
    
        #############################_Init_Node_start_##########################
        self.temp = 0
        self.home_value = 1280
        ##########################__INIT_NODE__############################## 
        rospy.init_node('LINE_FOLLOWER')
            
        ###########################__SUBSCRIBER__#############################
        
        mag_sub = rospy.Subscriber('/magnetline', String, self.Mag_callback)
        mag_sub_add = rospy.Subscriber('/magnetlineadd', String, self.Mag_add_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_callback)
        lift_sub = rospy.Subscriber('/lift', UInt32, self.lift_callback)
        key_sub = rospy.Subscriber('/key_press', String, self.key_callback)
        t_enc = rospy.Subscriber('/pos', Vector3, self.t_callback)
        pallet_pos_sub = rospy.Subscriber('/pospallet',Int32 , self.pallet_pos_callback)
        robot_charge_stt = rospy.Subscriber('/chargeCT',Int32 , self.robot_charge_stt_callback)
        robot_charge_ip = rospy.Subscriber('/chargeIP',String , self.robot_charge_IP_callback)
        
        ###########################__PUBLISH__##############################
        
        self.vel_pub = rospy.Publisher('twheel',Int32,queue_size = 100)
        self.ste_pub = rospy.Publisher('swheel',Int32,queue_size = 100)
        self.error_pub = rospy.Publisher('errorDetectedLine',Int32,queue_size = 100)
        self.lift_pub = rospy.Publisher('lift_control', String,queue_size = 100)
        self.program_pub = rospy.Publisher('linedetectioncallback', Int32,queue_size = 100)
        self.line_pos = rospy.Publisher('line_pos',Int32,queue_size = 100)#/charge_cmd
        self.charge_cmd = rospy.Publisher('/ctrlRobotHardware',Int32,queue_size = 100)
        self.charge_vol = rospy.Publisher('/charge_vol',Float32,queue_size = 100)
        self.debug = rospy.Publisher('/debug_line',Int32,queue_size = 100)
        
        #############################_Init_Node_end_##########################
        
        
        ##############################____CALLBACK-T____#########################
    
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
        #######################__MAGNETIC_ADD_SENSOR__########################
    
    def Mag_add_callback(self,msg):
        mag_sensor = str(msg.data)
        if int(mag_sensor[0]) == 0:
            self.mag_add_flag = int(mag_sensor[0])
        elif int(mag_sensor[1]) == 0:
            self.mag_add_flag = int(mag_sensor[1])
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
        elif data == 1208 :
            self.PID_enable = 6
        elif data == 1201:
            self.charge_stop = 1
        elif data == 1209:
            self.PID_enable = 7
            
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
        elif data == 'key a is press':
            self.charge_stop = 1
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
    ##########################__robot_charge_stt_callback__###########################
    def robot_charge_stt_callback(self,msg):
        data = msg.data
        if data == 5100:#power_on
            self.robot_charge_stt_var = 1
        elif data == 5101:#power_off 
            self.robot_charge_stt_var = 2
        else:
            pass

        ##########################__IP_CALLBACK__#########################
        
    def robot_charge_IP_callback(self,msg):
        self.host = msg.data
    
    ########################__USER-FUNCTION-START_#########################
    def take_pallet(self):
        if self.temp == 0:
            self.vel_pub.publish(200)
            self.ste_pub.publish(self.home_value)
            if 
    
    
    
    #########################__USER-FUNCTION-END_##########################
    
    ############################__MAIN-START__#############################
    
    def main(self):
        #r = rospy.Rate(20)
        while not rospy.is_shutdown():
            pid_.pid_cal(pos.position(),4,5)
#            if self.PID_enable == 2:
#                if self.status == 1:
#                    self.program_pub.publish(3203)
#                elif self.status == 2:
#                    self.program_pub.publish(3204)
#                else:
#                    pass
#                #self.s.close()
#                self.vel_pub.publish(0)
#                self.ste_pub.publish(self.home_value)
#                config.dict_     = None
#                config.line_flag = 0
#                config.pos_left  = 0
#                config.pos_right = 0
#                config.center    = 1
#                config.time      = 0
#                config.set_point = 8
#                for i in range(10):
#                    self.vel_pub.publish(0)
#                    self.ste_pub.publish(self.home_value)
#                self.PID_enable = 0
#            elif self.PID_enable == 1:
#                if self.server == 0:
#                    self.count_server += 1
#                    if self.count_server >= 30:
#                        self.server = 1
#                        self.count_server = 0
#                elif self.server == 1:
#                    #self.taking_pallet()
#                    pid_.pid_cal(pos.position(),4,5)
            
    
    ############################__MAIN-END__#############################
    
if __name__ == '__main__':
    run = line_follow()
    run.main()
