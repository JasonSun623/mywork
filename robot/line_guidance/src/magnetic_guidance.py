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
import socket
import json
import logging





class line_follow():

    ##############################____INIT____############################
    
    def __init__(self):
         ##############################
        self.host = "192.168.1.200"
        self.port = 8081
        ##############################
        self.laser_data              = []
        self.PID_enable              = 0
        self.lift_val                = 3
        self.stop_flag               = 0
        self.stop_encoder            = 0
        self.flag_1                  = 0
        self.stop_flag_laser         = 0
        self.flag                    = 0
        self.flag_laser              = 0
        self.mag_ss                  = []
        self.mag_ss_front            = []
        self.error                   = 0
        self.mag_add_flag            = 3
        self.set_point               = 8
        self.last_proportional       = 0
        self.integral                = 0
        self.home_value              = 5200
        self.last_ster_val           = 0
        self.full_line_flag          = 0
        self.angle                   = 0
        self.temp                    = 0
        self.pos_left                = 0
        self.pos_right               = 0
        self.pos_stop                = 0
        self.center                  = 1
        self.list_a                  = [7,6,5]
        self.list_b                  = [10,11,12]
        self.stear_enc               = 0
        self.t_enc                   = 0
        self.temp_1                  = 0
        self.time                    = 0
        self.cross_detect            = 0
        self.cross_front_detect      = 0
        self.take_pallet             = 0
        self.status                  = 0
        self.temp_2                  = 0 
        self.count                   = 0
        self.count_1                 = 0
        self.count_2                 = 0
        self.count_3                 = 0
        self.count_4                 = 0
        self.count_5                 = 0
        self.count_6                 = 0
        self.count_7                 = 0
        self.count_8                 = 0
        self.count_9                 = 0
        self.count_11                = 0
        self.count_12                = 0
        self.temp_enc                = 0
        self.now_encoder             = 0
        self.last_encoder            = 0
        self.last_encoder_1          = 0
        self.last_encoder_2          = 0
        self.last_encoder_3          = 0
        self.last_encoder_4          = 0
        self.loss_line_flag          = 0
        self.loss_line_flag_1        = 0
        self.loss_line_temp          = 0
        self.loss_line_stt           = 0
        self.count_lane              = 0
        self.loss_line_temp_1        = 0
        self.loss_line_temp_2        = 0
        self.loss_line_temp_3        = 0
        self.loss_line_temp_4        = 0
        self.out_take_pallet         = 0
        self.out_put_pallet          = 0
        self.loss_line_temp_6        = 0
        self.count_magss             = 0
        self.count_front_magss       = 0
        self.count_3                 = 0
        self.line_flag               = 0
        self.flag_2                  = 0
        self.mag_left_value          = 2
        self.mag_right_value         = 2
        self.bay_count              = 0
        self.row_countout          = 0
        self.row_count             = 0
        self.no_line_flag            = 0
        self.no_line_flag_front      = 0
        self.turn_flag               = 0
        self.balance_flag            = 0
        self.dir_main_temp           = 0
        self.dir_main_count          = 0
        self.traffic_flag            = 2
        #self.bay = 1
        #self.row = 2
        self.charger_ready           = 0
        self.turn_off_pc_flag        = 0
        self.pallet                  = 0
        self.bay                     = 0
        self.has_sub_line            = None
        self.dir_main                = None
        self.dir_sub                 = None
        self.line_ord                = 0
        self.line_ord_count          = 0
        self.row                     = 0
        self.dir_out                 = 0
        self.count_10                = 0 
        self.charger_flag            = 0
        self.file_count              = 0
        self.encoder_var             = 2.2
#        self.logger = logging.getLogger('line_folow')
#        self.hdlr = logging.FileHandler('log_line/log.txt')
#        self.formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
#        self.hdlr.setFormatter(self.formatter)
#        self.logger.addHandler(self.hdlr) 
#        self.logger.setLevel(logging.INFO)
        ##########################__INIT_NODE__############################## 
        rospy.init_node('LINE_FOLLOWER')
        
        ###########################__SUBSCRIBER__#############################
        mag_sub = rospy.Subscriber('/magnetback', String, self.Mag_callback)
        mag_sub___ = rospy.Subscriber('/magnetfront', String, self.Mag_front_callback)
        mag_sub_ = rospy.Subscriber('/magnetleft', String, self.Mag_left_callback)
        mag_sub__ = rospy.Subscriber('/magnetright', String, self.Mag_right_callback)
        laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_charger_callback)
        lift_sub = rospy.Subscriber('/lift', UInt32, self.lift_callback)
        key_sub = rospy.Subscriber('/key_press', String, self.key_callback)
        stear_enc = rospy.Subscriber('/pos', Vector3, self.stear_callback)
#        t_enc = rospy.Subscriber('/pos', Vector3, self.t_callback)
        pallet_pos_sub = rospy.Subscriber('/pospallet',Int32 , self.pallet_pos_callback)
        command_robot = rospy.Subscriber('/cmdAreaPallet',String , self.server_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_cmd_callback)
        traffic_sub = rospy.Subscriber('ctrlRobotDriving', Int32, self.traffic_cmd_callback)
        ###########################__PUBLISH__##############################
        self.vel_pub = rospy.Publisher('twheel',Int32,queue_size = 10)
        #self.vel_pub = rospy.Publisher('fwheel_vtarget',Float32,queue_size = 100)
        self.ste_pub = rospy.Publisher('swheel',Int32,queue_size = 10)
        self.error_pub = rospy.Publisher('errorDetectedLine',Int32,queue_size = 10)
        self.lift_pub = rospy.Publisher('lift_control', String,queue_size = 10)
        self.program_pub = rospy.Publisher('linedetectioncallback', Int32,queue_size = 10)
        self.line_pos = rospy.Publisher('line_pos',Int32,queue_size = 10)
        self.line_pub = rospy.Publisher('line_enable',Int32,queue_size = 10)
#        self.reset_run_pub = rospy.Publisher('reset_run_button',Int32,queue_size = 10)
        self.charge_cmd = rospy.Publisher('/ctrlRobotHardware',Int32,queue_size = 10)
        
        
    ##############################____CALL_BACK____#########################
            
        #######################__MAGNETIC_SENSOR__########################
    
    def Mag_callback(self,msg):
        mag_sensor = str(msg.data)
        mag_sensor_list = np.array(list(mag_sensor),dtype=np.int)
        mag_value_not = np.logical_not(mag_sensor_list)
        self.mag_ss = map(int,mag_value_not)
        self.count_magss = mag_sensor.count('0')
        if self.count_magss >= 15:
            self.cross_detect = 1
        else:
            self.cross_detect = 0
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            self.no_line_flag = 1
        else:
            self.no_line_flag = 0
    def Mag_front_callback(self,msg):
        mag_sensor = str(msg.data)
        mag_sensor_list = np.array(list(mag_sensor),dtype=np.int)
        mag_value_not = np.logical_not(mag_sensor_list)
        self.mag_ss_front = map(int,mag_value_not)
        self.count_front_magss = mag_sensor.count('0')
        if self.count_front_magss >= 15:
            self.cross_front_detect = 1
        else:
            self.cross_front_detect = 0
        if self.mag_ss_front == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            self.no_line_flag_front = 1
        elif self.mag_ss_front == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
            self.full_line_flag = 1     
        else:
            self.full_line_flag = 0
            self.no_line_flag_front = 0
        # REMEMBER CHECK THIS FUNCTION(WRITE NEW 14/11/2017 BY DONGHO)
        #######################__MAGNETIC_ADD_SENSOR__########################
    
    def Mag_left_callback(self,msg):
        mag_sensor = str(msg.data)
        if int(mag_sensor[0]) == 0 and int(mag_sensor[1]) == 1:
            self.mag_left_value = 0
        elif int(mag_sensor[0]) == 1 and int(mag_sensor[1]) == 0:
            self.mag_left_value = 1
        else:
            self.mag_left_value = 2
    def Mag_right_callback(self,msg):
        mag_sensor = str(msg.data)
        if int(mag_sensor[0]) == 0 and int(mag_sensor[1]) == 1:
            self.mag_right_value = 0
        elif int(mag_sensor[0]) == 1 and int(mag_sensor[1]) == 0:
            self.mag_right_value = 1
        else:
            self.mag_right_value = 2
        #######################__LASER__########################
        
    def laser_callback(self,msg):
        del self.laser_data[:]
        for i in range(0,180):
            self.laser_data.append(msg.ranges[i])
        self.stop_flag_laser = list(filter(lambda x: x > 0 and x < 0.9  ,self.laser_data))
        self.stop_flag_laser = len(self.stop_flag_laser)
        
        ##########################__SERVER__#########################
    def server_charger_callback(self,msg):  
        data = msg.data
        if data == 1206:
            self.PID_enable = 6   
        elif data == 1207:
            self.PID_enable = 7                  
#        data = msg.data
#        if data == 1203:
#            self.PID_enable = 1
#        elif data == 1200:
#            self.PID_enable = 2
#            self.status = 0
#        elif data == 1204 :
#            self.PID_enable = 3
#        elif data == 1206:
#            self.PID_enable = 4
#        elif data == 1207 :
#            self.PID_enable = 5
            
        ##########################__LIFT__###########################
    
    def lift_callback(self,msg):
        self.lift_val = msg.data
        
    ##########################__TRAFFIC__###########################
    
    def traffic_cmd_callback(self,msg):
        data = msg.data
        if data == 0:
            self.traffic_flag = 0
        elif data == 2:
            self.traffic_flag = 2
        else:
            pass
    
    ##########################__JSON__###########################
    
    def server_callback(self,msg):
        data                = msg.data
        dict_data           = json.loads(data)
        self.pallet         = dict_data["pallet"] 
        self.dir_main       = dict_data["dir_main"]
        self.has_sub_line   = dict_data["hasSubLine"]
        self.dir_sub        = dict_data["dir_sub"]
        self.row            = dict_data["row"] + 1
        self.dir_out        = dict_data["dir_out"] + 1
        try:
            self.line_ord       = dict_data["line_ord"] + 1
        except:
            self.line_ord = 1
        
        if dict_data["hasSubLine"] == "yes" and dict_data["dir_sub"] == 1:
            if dict_data["bay"] == 0:
                self.bay = (dict_data["bay"] + 2)
            else:
                self.bay = (dict_data["bay"] + 1) * 2
        else:
            if dict_data["bay"] == 0:
                self.bay   = (dict_data["bay"] + 1)
            else:
                self.bay   = (((dict_data["bay"] +1 ) * 2) - 1)
        print(dict_data)
        if self.pallet == 2:
            self.PID_enable = 3
        else:
            self.PID_enable = 1
        ###print(dict_data["row"],type(dict_data["row"]))
    def server_cmd_callback(self,msg):
        data = msg.data
#        if data == 1206:
#            self.charger_ready = 1
#        elif data == 1207:
#            self.PID_enable = 4
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
        self.t_enc = msg.x
        
    ##########################__t_callback__###########################
#    def t_callback(self,msg):
#        self.t_enc = msg.x
        
    ##########################__POSITION_CALLBACK__###########################
    def pallet_pos_callback(self,msg):
        data = msg.data
        if data == 1205:
            self.pos_stop = 1
        else:
            self.pos_stop = 0
        if data == 1214:
            self.PID_enable = 5
        elif data == 1213:
            self.PID_enable = 4

    #############################____FUNCTIONs____##########################
        ####################__error_calculation__#####################
        
    def position(self,mag_ss):
        ss_sum = 0
        ss_aveg = 0
        for i in range(0,len(mag_ss)) :
            ss_aveg += mag_ss[i] * i
        ss_sum += self.count_magss
        if ss_sum == 0:
            pos = 8
            self.line_flag = 1
        else:
            ###print("pos____",ss_aveg,"pos***",ss_sum)
            pos = (ss_aveg/ss_sum) + 1
            self.line_flag = 0
        if pos < 7:
            self.pos_left = 1
            self.pos_right = 0
            self.center = 0
        elif pos > 9:
            self.pos_left = 0
            self.pos_right = 1
            self.center = 0
        return pos
    def position_front(self,mag_ss):
        ss_sum = 0
        ss_aveg = 0
        for i in range(0,len(mag_ss)) :
            ss_aveg += mag_ss[i] * i
        ss_sum += self.count_front_magss
        ###print("ss_aveg= ",ss_aveg,"ss_sum = ",self.count_front_magss)
        if ss_sum == 0:
            pos = 8
            self.line_flag = 1
        else:
            ###print("pos____",ss_aveg,"pos***",ss_sum)
            pos = (ss_aveg/ss_sum) + 1
            self.line_flag = 0
        if pos < 7:
            self.pos_left = 1
            self.pos_right = 0
            self.center = 0
        elif pos > 9:
            self.pos_left = 0
            self.pos_right = 1
            self.center = 0
        ##print("pos = ",pos)
        return pos
        
        #####################__Timer__#####################
    def timer(self,pos,loss_line_flag):
        ###print "self.loss_line_flag_1 = " ,self.loss_line_flag_1,"pos = ",pos
        if pos >= 4 and pos < 6:
            self.time = -60#0.9
        elif pos >= 6 and pos < 7:
            self.time = -80#0.6
        elif pos < 4 and pos > 2:
            self.time = -80#1.4
        elif pos <= 2:
            self.time = -90
        elif pos > 9 and pos <= 11:
            self.time = -60#0.6
        elif pos > 11 and pos <= 12:
            self.time = -80#0.9
        elif pos > 12 and pos <= 13 :
            self.time = -80#1.4
        elif pos > 13 and pos <= 15 :
            self.time = -80#1.4
        elif pos >15 :
            self.time = -90
        #self.line_pos.publish(pos)
        ###print "pos_1 = ",pos
        
        
        
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
        ###print "pos = ",pos,"ster_value = ",ster_value
        #time.sleep(0.05)
        return ster_value
        
    #####################__angle_control__#####################
    def angle_controll(self,speed):
        if self.full_line_flag == 1:
            self.vel_pub.publish(speed)
            self.ste_pub.publish(self.angle)
        else:
            pos = self.position(self.mag_ss)
            turning_value = self.pid_cal(self.position(self.mag_ss),30,150)#35,5
            ###print turning_value,int(turning_value)
            self.angle = self.home_value + turning_value
            self.angle = round(self.angle)
            self.angle = int(self.angle)
            if self.angle > self.home_value:
                self.angle += 21
            if self.angle < self.home_value :
                self.angle += 45#2
            else:
                self.angle = self.angle
            ##print "self.angle",self.angle,"pos",pos
            if self.angle > self.home_value + 150 and self.angle <= self.home_value + 250 :
                self.angle +=45
                if speed < 0:
                    speed = speed + 50
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            elif self.angle > self.home_value + 250 and self.angle <= self.home_value + 350 :
                self.angle = self.angle + 250
                #self.angle = 750
                if speed < 0:
                    speed = speed + 150
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            elif self.angle > self.home_value + 350 :
                self.angle = self.angle + 300
                #self.angle = 750
                if speed < 0:
                    speed = speed + 150
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            elif self.angle < self.home_value - 100 and self.angle >= self.home_value - 200:
                self.angle += -10#10
                if speed < 0:
                    speed = speed + 50
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            elif self.angle < self.home_value - 200 and self.angle >= self.home_value - 300 : 
                self.angle = self.angle - 200
                if speed < 0:
                    speed = speed + 150
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            elif self.angle < self.home_value - 300 : 
                self.angle = self.angle - 250
                if speed < 0:
                    speed = speed + 150
                elif speed == 0:
                    speed = 0
                else:
                    speed = speed - 200
            ##print "speed",speed, "angle",self.angle#,"pos",pos
            self.vel_pub.publish(speed)
            self.ste_pub.publish(self.angle)
            if pos >= 15:
                self.balance_flag = 1
                self.timer(pos,self.loss_line_flag_1)
            elif pos <= 2:
                self.balance_flag = 1
                self.timer(pos,self.loss_line_flag_1)
            else:
                pass
    ###############################################################################
    
    def angle_controll_front(self,speed):
        pos = self.position_front(self.mag_ss_front)
        ###print("pos______",pos)
        turning_value = self.pid_cal(self.position_front(self.mag_ss_front),30,120)#35,5
        ###print turning_value,int(turning_value)
        self.angle = self.home_value - turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        if self.angle > self.home_value:
            self.angle += 0
        if self.angle < self.home_value :
            self.angle -= 25#2
        else:
            self.angle = self.angle
        ##print "self.angle",self.angle
        if self.angle > self.home_value + 150 and self.angle <= self.home_value + 250 :
            self.angle +=0
            if speed < 0:
                speed = speed + 50
            elif speed == 0:
                speed = 0
            else:
                speed = speed - 50
        elif self.angle > 5450:
            self.angle = self.angle + 50
            #self.angle = 750
            if speed < 0:
                speed = speed + 50
            elif speed == 0:
                speed = 0
            else:
                speed = speed - 50
        elif self.angle < self.home_value - 100 and self.angle >= self.home_value - 200:
            self.angle -= 20#10
            if speed < 0:
                speed = speed + 50
            elif speed == 0:
                speed = 0
            else:
                speed = speed - 200
        elif self.angle < self.home_value - 200 : 
            self.angle = self.angle - 200
            if speed < 0:
                speed = speed + 50
            elif speed == 0:
                speed = 0
            else:
                speed = speed - 50
        if pos > 13:
            self.angle = self.angle - 220
        elif pos < 4:
            self.angle = self.angle + 220
        else:
            pass
            
        ##print "speed",speed, "angle",self.angle#,"pos",pos
        self.vel_pub.publish(speed)
        self.ste_pub.publish(self.angle)

        #####################__Loss_line_left__#####################
    def loss_line(self):
        #self.loss_line_flag_1 = 1
        if self.loss_line_temp == 0:
            self.last_encoder = -(self.t_enc)
            self.loss_line_temp = 1
        elif self.loss_line_temp == 1:
            ##print ((self.last_encoder) + (self.t_enc))
            if self.has_sub_line == "no" and self.line_ord == 1:
                if self.line_ord_count == self.line_ord:
                    self.loss_line_temp = 3
                    self.loss_line_temp_3 = 0 
                else:
                    if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
                        self.line_ord_count += 1
                        self.loss_line_temp_3 = 1
                    elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
                        pass
                    else:
                        self.loss_line_temp_3 = 0 
                    self.vel_pub.publish(-1100)
                    self.ste_pub.publish(self.home_value)
                if ((self.last_encoder) + (self.t_enc)) < (-2300*self.encoder_var) :
                    for i in range(30):
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
                    self.PID_enable = 2
            elif self.has_sub_line == "no" and self.line_ord == 2:
                if self.line_ord_count == self.line_ord:
                    self.loss_line_temp = 3
                    self.loss_line_temp_3 = 0 
                else:
                    if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
                        self.line_ord_count += 1
                        self.loss_line_temp_3 = 1
                    elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
                        pass
                    else:
                        self.loss_line_temp_3 = 0 
                    self.vel_pub.publish(-1100)
                    self.ste_pub.publish(self.home_value)
                if ((self.last_encoder) + (self.t_enc)) < -2300*self.encoder_var  :
                    for i in range(30):
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
                    self.PID_enable = 2
            else:
                if self.count_magss > 3 :
                    self.loss_line_temp = 3
                elif ((self.last_encoder) + (self.t_enc)) < -2300*self.encoder_var :
                    for i in range(30):
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
                    self.PID_enable = 2
                else:
                    self.vel_pub.publish(-1100)
                    self.ste_pub.publish(self.home_value)
#        elif self.loss_line_temp == 2:
#            for i in range(30):
#                self.vel_pub.publish(0)
#                self.ste_pub.publish(self.home_value)
#            self.loss_line_temp = 3
        elif self.loss_line_temp == 3:
            if self.dir_main == 1:
                if self.count_2 == 0:
                    self.last_encoder_3 = -(self.t_enc)
                    self.count_2 = 1
                elif self.count_2 == 1:
                    if ((self.last_encoder_3) + (self.t_enc)) < -212*self.encoder_var:
                        for i in range(5):
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(self.home_value)
                        for i in range(20):
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(2000)
                            self.count_2 = 0
                            self.last_encoder_1 = -(self.t_enc)
                            self.loss_line_temp = 4
                    else:
                        self.vel_pub.publish(-1150)
                        self.ste_pub.publish(self.home_value)
            elif self.dir_main == 2:
                if self.count_2 == 0:
                    self.last_encoder_3 = -(self.t_enc)
                    self.count_2 = 1
                elif self.count_2 == 1:
                    if ((self.last_encoder_3) + (self.t_enc)) < -50*self.encoder_var:
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
                        self.count_2 = 2
                        self.last_encoder_3 = -(self.t_enc)
                    else:
                        self.vel_pub.publish(-1100)
                        self.ste_pub.publish(self.home_value)
                elif self.count_2 == 2:
#                    #print("(self.last_encoder_3) + (self.t_enc)",(self.last_encoder_3) + (self.t_enc))
                    if ((self.last_encoder_3) + (self.t_enc)) > 5*self.encoder_var:
                        for i in range(20):
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(self.home_value)
                        for i in range(20):
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(2000)
                        self.count_2 = 0
                        self.loss_line_temp = 4
                        self.last_encoder_1 = -(self.t_enc)
                    else:
                        self.vel_pub.publish(1150)
                        self.ste_pub.publish(self.home_value)
        elif self.loss_line_temp == 4:
            if self.dir_main == 1:
                if ((self.last_encoder_1) + (self.t_enc)) > 1100*self.encoder_var:
                    self.loss_line_temp = 5
                    self.vel_pub.publish(1000)
                    self.ste_pub.publish(2000)
                else:
                    self.vel_pub.publish(1300)
                    self.ste_pub.publish(2000)
            elif self.dir_main == 2:
                if ((self.last_encoder_1) + (self.t_enc)) < -1100*self.encoder_var:
                    self.loss_line_temp = 5
                    self.dir_main_temp = 2
                    self.vel_pub.publish(-1000)
                    self.ste_pub.publish(2000)
                else:
                    self.vel_pub.publish(-1300)
                    self.ste_pub.publish(2000)
        elif self.loss_line_temp == 5:
            if self.dir_main == 1:
                if self.mag_left_value == 0:
                    self.loss_line_temp = 6
                else:
                    ###print(((self.last_encoder_1) + (self.t_enc)))
                    self.vel_pub.publish(1000)  #left
                    self.ste_pub.publish(2000)
            elif self.dir_main == 2:
                if self.mag_left_value == 0:
                    self.loss_line_temp = 6
                else:
                    ###print(((self.last_encoder_1) + (self.t_enc)))
                    self.vel_pub.publish(-1000)  #left
                    self.ste_pub.publish(2000)
            else:
                pass
        elif self.loss_line_temp == 6:
            self.count_11 += 1
            if self.count_11 < 10:
                self.vel_pub.publish(0)
                self.ste_pub.publish(2000)
            elif self.count_11 > 10 and self.count_11 < 30:
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
            elif self.count_11 > 30:
                self.loss_line_flag = 1
                self.loss_line_temp_2 = 1
                self.count_lane = 2
                self.loss_line_temp = 0
                self.count_11 = 0
            else:
                pass
                
        ###########################################################
        
    def loss_line_charger(self):
        #self.loss_line_flag_1 = 1
        if self.loss_line_temp == 0:
            self.last_encoder = -(self.t_enc)
            self.loss_line_temp = 1
        elif self.loss_line_temp == 1:
            if self.count_magss > 3 :
                self.loss_line_temp = 2
            elif ((self.last_encoder) + (self.t_enc)) < -2300*self.encoder_var :
                for i in range(30):
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(self.home_value)
                self.PID_enable = 2
            else:
                self.vel_pub.publish(-1100)
                self.ste_pub.publish(self.home_value)
        elif self.loss_line_temp == 2:
            #time.sleep(1)
            self.vel_pub.publish(0)
            self.ste_pub.publish(self.home_value)
            self.loss_line_temp = 3
        elif self.loss_line_temp == 3:
            for i in range(10):
                self.vel_pub.publish(0)  #left
                self.ste_pub.publish(200)
            self.loss_line_temp = 4
            self.last_encoder_1 = -(self.t_enc)
        elif self.loss_line_temp == 4:
            ##print("encoder_charger",((self.last_encoder_1) + (self.t_enc)))
            if ((self.last_encoder_1) + (self.t_enc)) < -800*self.encoder_var:
                self.loss_line_temp = 5
                self.vel_pub.publish(-1300)
                self.ste_pub.publish(2000)
            else:
                self.vel_pub.publish(-1300)
                self.ste_pub.publish(2000)
        elif self.loss_line_temp == 5:
            ##print(self.position(self.mag_ss_front))
            if self.position(self.mag_ss_front) > 6 and self.position(self.mag_ss_front) < 10 and self.no_line_flag_front == 0 :
            #ssif self.position(self.mag_ss_front) > 3 and self.position(self.mag_ss_front) < 15 and self.no_line_flag_front == 0 :
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
                self.loss_line_flag = 1
                self.loss_line_temp_2 = 1
                self.count_lane = 2
                self.loss_line_temp = 0
            else:
                ##print(((self.last_encoder_1) + (self.t_enc)))
                self.vel_pub.publish(-1000)  #left
                self.ste_pub.publish(2000)
    
        #####################__taking_pallet__#####################
    def taking_pallet(self):
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
            self.count_lane = 1
        if self.mag_ss == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
            self.count_lane = 1
        elif self.count_lane == 1 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0:
            self.loss_line()
        elif self.count_lane == 2 and self.loss_line_flag == 1 and self.loss_line_temp_2 == 1:
            #self.loss_line_flag_1 = 0
            if self.mag_flag(self.mag_ss) == 1 and self.flag_2 == 0:
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
                self.count_3 += 1
                if self.count_3 > 30:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(self.home_value)
                    self.error_pub.publish(3215)
                    self.PID_enable = 2
                    self.status = 3
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(5200)
                ##print "Can not find lane to run "                
            else:
                if self.temp_2 == 0:
                    self.count += 1
                    if self.count >= 20:
                        self.temp_2 = 1
                        self.count = 0
                    else:
                        pos = self.position(self.mag_ss)
                        self.timer(pos,self.loss_line_flag_1)
                else:
                    #self.flag_1 = 1
                    pos = self.position(self.mag_ss)
                    if self.stop_flag == 1:
                        if self.take_pallet == 0:
                            if self.has_sub_line == "yes":
                                self.take_pallet = 1
                            else:
                                self.take_pallet = 5
                            ##########uncomment here when release#######
                            #if self.lift_val == 1 or self.lift_val == 0:
                            #    self.vel_pub.publish(0)
                            #    self.ste_pub.publish(5200)
                            #    self.lift_pub.publish("lift_down")
                            #    if self.lift_val == 2:
                            #        self.lift_pub.publish("lift_stop")  
                            #        self.take_pallet = 1
                            #else:
                            #    self.take_pallet = 1
                            
                        elif self.take_pallet == 1:
                            if self.cross_detect == 1 :
                                if self.loss_line_temp_3 == 0  :
                                    self.bay_count += 1
                                    self.loss_line_temp_3 = 1
                                elif self.loss_line_temp_3 == 1:
                                    pass
                            else:
                                self.loss_line_temp_3 = 0
                            #print("take_pallet == 1",self.bay_count)
                            if self.bay_count == self.bay :#uncomment here when done
                                self.stop_encoder = -(self.t_enc)
                                ###print "stop_encoder = ",self.stop_encoder
                                self.take_pallet = 2
                                self.turn_flag = 0
                                #self.pos_stop = 0
                            else:
                                if self.balance_flag == 1:
                                    if self.turn_flag == 0:
                                        if self.pos_left == 1:
#                                            if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
#                                                self.bay_count += 1
#                                                self.loss_line_temp_3 = 1
#                                            elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
#                                                pass
#                                            else:
#                                                self.loss_line_temp_3 = 0
                                            ###print " move from the left",pos,self.count_magss
                                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                                self.turn_flag = 1
                                                self.temp_1 = 1
                                                self.vel_pub.publish(-1000)#21h-11/12/2017
                                                self.ste_pub.publish(self.home_value)
                                                self.now_encoder = -self.t_enc
                                                self.loss_line_temp_3 = 0
                                            else:
                                                self.angle_controll(-1200)
                                                #print("111111111")
#                                            if self.cross_detect == 1 and self.bay_count == self.bay :#uncomment here when done
#                                                #self.stop_encoder = -(self.t_enc)
#                                                ###print "stop_encoder = ",self.stop_encoder
#                                                self.take_pallet = 2
#                                                self.turn_flag = 0
                                        elif self.pos_right == 1:
#                                            if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
#                                                self.bay_count += 1
#                                                self.loss_line_temp_3 = 1
#                                            elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
#                                                pass
#                                            else:
#                                                self.loss_line_temp_3 = 0
                                                    #self.temp = self.temp + 1
                                            ###print "move from the right",pos,self.count_magss
                                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                                self.vel_pub.publish(-1000)
                                                self.ste_pub.publish(self.home_value)
                                                self.temp_1 = 2
                                                self.turn_flag = 1
                                                self.now_encoder = -self.t_enc
                                                self.loss_line_temp_3 = 0
                                            else:
                                                self.angle_controll(-1200)
                                                #print("111111111")
#                                            if self.cross_detect == 1 and self.bay_count == self.bay :#uncomment here when done
#                                                #self.stop_encoder = -(self.t_enc)
#                                                ###print "stop_encoder = ",self.stop_encoder
#                                                self.take_pallet = 2
#                                                self.turn_flag = 0
                                    elif self.turn_flag == 1:
#                                        if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
#                                            self.bay_count += 1
#                                            self.loss_line_temp_3 = 1
#                                        elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
#                                            #print("okokok")
#                                            pass
#                                        else:
#                                            self.loss_line_temp_3 = 0
                                        if self.temp_1 == 1 :
                                            if ((self.now_encoder) + (self.t_enc)) <= (self.time + 15)*self.encoder_var :
                                                self.temp_1 = 0
                                                self.balance_flag = 0
                                                self.turn_flag = 0
                                                self.loss_line_temp_3 = 0
                                            else:
                                                self.loss_line_temp_3 = 0
                                                self.vel_pub.publish(-1000)  #left
                                                self.ste_pub.publish(8300)
                                        elif self.temp_1 == 2:
                                            if ((self.now_encoder) + (self.t_enc)) <= (self.time + 15)*self.encoder_var :
                                                self.temp_1 = 0
                                                self.balance_flag = 0
                                                self.turn_flag = 0
                                                self.loss_line_temp_3 = 0
                                            else:
                                                self.loss_line_temp_3 = 0
                                                self.vel_pub.publish(-1000)  #right
                                                self.ste_pub.publish(2000)
#                                        if self.cross_detect == 1 and self.bay_count == self.bay :#uncomment here when done
#                                            self.stop_encoder = -(self.t_enc)
#                                            ###print "stop_encoder = ",self.stop_encoder
#                                            self.take_pallet = 2
#                                            self.turn_flag = 0
                                else:          
#                                    if self.cross_detect == 1 and self.loss_line_temp_3 == 0  :
#                                        self.bay_count += 1
#                                        self.loss_line_temp_3 = 1
#                                    elif self.cross_detect == 1 and self.loss_line_temp_3 == 1:
#                                        #print("okokok")
#                                        pass
#                                    else:
#                                        self.loss_line_temp_3 = 0
                                    self.flag = 0
                                    if self.bay_count == (self.bay - 1):
                                        self.angle_controll(-1150)
                                    else:
                                        self.angle_controll(-1350)  
#                                        #print("111111111",self.loss_line_temp_3)
#                                        #print("self.row_count111111111 = ",self.bay_count)   
#                                    if self.cross_detect == 1 and self.bay_count == self.bay :#uncomment here when done
#                                        self.stop_encoder = -(self.t_enc)
#                                        ###print "stop_encoder = ",self.stop_encoder
#                                        self.take_pallet = 2
#                                        self.turn_flag = 0
                        elif self.take_pallet == 2:       
                            #print("here333333333333")
                            self.flag_2 = 1
                            if self.dir_sub == 1:
                                if self.count_2 == 0:
                                    self.last_encoder_3 = -(self.t_enc)
                                    self.count_2 = 1
                                elif self.count_2 == 1:
                                    if ((self.last_encoder_3) + (self.t_enc)) < -212*self.encoder_var:
                                        for i in range(5):
                                            self.vel_pub.publish(0)
                                            self.ste_pub.publish(self.home_value)
                                        for i in range(20):
                                            self.vel_pub.publish(0)
                                            self.ste_pub.publish(2000)
                                        self.count_2 = 0
                                        self.take_pallet = 3
                                        print("self.bay_count",self.bay_count)
                                    else:
                                        self.vel_pub.publish(-1150)
                                        self.ste_pub.publish(self.home_value)
                            elif self.dir_sub == 2:
                                if self.count_2 == 0:
                                    self.last_encoder_3 = -(self.t_enc)
                                    self.count_2 = 1
                                elif self.count_2 == 1:
                                    if ((self.last_encoder_3) + (self.t_enc)) < -50*self.encoder_var:
                                        self.vel_pub.publish(0)
                                        self.ste_pub.publish(self.home_value)
                                        self.count_2 = 2
                                        self.last_encoder_3 = -(self.t_enc)
                                    else:
                                        self.vel_pub.publish(-1100)
                                        self.ste_pub.publish(self.home_value)
                                elif self.count_2 == 2:
                                    if ((self.last_encoder_3) + (self.t_enc)) > 5*self.encoder_var:
                                        for i in range(5):
                                            self.vel_pub.publish(0)
                                            self.ste_pub.publish(self.home_value)
                                        for i in range(20):
                                            self.vel_pub.publish(0)
                                            self.ste_pub.publish(2000)
                                        self.count_2 = 0
                                        self.take_pallet = 3
                                    else:
                                        self.vel_pub.publish(1150)
                                        self.ste_pub.publish(self.home_value)
                        elif self.take_pallet == 3:
                            self.flag_2 = 1
                            self.count_5 += 1
                            
                            if self.count_5 < 150:
                                if self.dir_sub == 1:
                                    self.vel_pub.publish(1100)
                                    self.ste_pub.publish(2000)
                                elif self.dir_sub == 2:
                                    self.vel_pub.publish(-1100)
                                    self.ste_pub.publish(2000)
                                else:
                                    pass
                            elif self.count_5 >= 150 and self.count_5 < 270:
                                if self.dir_sub == 1:
                                    self.vel_pub.publish(1300)
                                    self.ste_pub.publish(2000)
                                elif self.dir_sub == 2:
                                    self.vel_pub.publish(-1300)
                                    self.ste_pub.publish(2000)
                                else:
                                    pass
                            else:
                                if self.dir_sub == 1:
                                    if self.mag_left_value == 0:
                                        self.vel_pub.publish(0)
                                        self.ste_pub.publish(self.home_value)
                                        self.temp_2 = 0
                                        #self.stop_flag = 0
                                        self.take_pallet = 4
                                        self.count_5 = 0
                                        self.temp_enc = -(self.t_enc)
                                    else:
                                        self.vel_pub.publish(1100)
                                        self.ste_pub.publish(2000)
                                elif self.dir_sub == 2:
                                    if self.mag_left_value == 0:
                                        self.vel_pub.publish(0)
                                        self.ste_pub.publish(self.home_value)
                                        self.temp_2 = 0
                                        self.take_pallet = 4
                                        self.count_5 = 0
                                        self.temp_enc = -(self.t_enc)
                                    else:
                                        self.vel_pub.publish(-1100)
                                        self.ste_pub.publish(2000)
                        elif self.take_pallet == 4:
                            #print(((self.temp_enc) + (self.t_enc)))
                            if ((self.temp_enc) + (self.t_enc)) < -700*self.encoder_var:
                                self.take_pallet = 5
                                self.count_9 = 0
                                self.flag_2 = 0
                            else:
                                self.angle_controll(-1200)
                        elif self.take_pallet == 5:
                            if self.cross_detect == 1 :
                                if self.loss_line_temp_4 == 0  :
                                    self.row_count += 1
                                    self.loss_line_temp_4 = 1
                                elif self.loss_line_temp_4 == 1:
                                    pass
                            else:
                                self.loss_line_temp_4 = 0
                            #print("self.row_count",self.row_count)
                            if self.row_count == self.row :#uncomment here when done
                                self.flag_2 = 1
                                self.stop_encoder = -(self.t_enc)
                                ###print "stop_encoder = ",self.stop_encoder
                                self.take_pallet = 6
                                #self.pos_stop = 0
                            else:
                                if self.balance_flag == 1:
                                    if self.turn_flag == 0:
                                        if self.pos_left == 1:
#                                            if self.cross_detect == 1 and self.loss_line_temp_4 == 0  :
#                                                self.row_count += 1
#                                                self.loss_line_temp_4 = 1
#                                            elif self.cross_detect == 1 and self.loss_line_temp_4 == 1:
#                                                pass
#                                            else:
#                                                self.loss_line_temp_4 = 0
                                            ###print " move from the left",pos,self.count_magss
                                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                                self.turn_flag = 1
                                                self.temp_1 = 1
                                                self.vel_pub.publish(-1000)#21h-11/12/2017
                                                self.ste_pub.publish(self.home_value)
                                                self.now_encoder = -self.t_enc
                                                self.loss_line_temp_4 = 0
                                            else:
                                                self.angle_controll(-1200)
                                                #print("111111111")
#                                            if self.cross_detect == 1 and self.row_count == self.row :#uncomment here when done
#                                                #self.stop_encoder = -(self.t_enc)
#                                                ###print "stop_encoder = ",self.stop_encoder
#                                                self.flag_2 = 1
#                                                self.take_pallet = 6
                                        elif self.pos_right == 1:
#                                            if self.cross_detect == 1 and self.loss_line_temp_4 == 0  :
#                                                self.row_count += 1
#                                                self.loss_line_temp_4 = 1
#                                            elif self.cross_detect == 1 and self.loss_line_temp_4 == 1:
#                                                pass
#                                            else:
#                                                self.loss_line_temp_4 = 0
                                                    #self.temp = self.temp + 1
                                            ###print "move from the right",pos,self.count_magss
                                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                                self.vel_pub.publish(-1000)
                                                self.ste_pub.publish(self.home_value)
                                                self.temp_1 = 2
                                                self.turn_flag = 1
                                                self.now_encoder = -self.t_enc
                                                self.loss_line_temp_4 = 0
                                            else:
                                                self.angle_controll(-1200)
                                                #print("111111111")
#                                            if self.cross_detect == 1 and self.row_count == self.row :#uncomment here when done
#                                                #self.stop_encoder = -(self.t_enc)
#                                                ###print "stop_encoder = ",self.stop_encoder
#                                                self.flag_2 = 1
#                                                self.take_pallet = 6
                                    elif self.turn_flag == 1:
#                                        if self.cross_detect == 1 and self.loss_line_temp_4 == 0  :
#                                            self.row_count += 1
#                                            self.loss_line_temp_4 = 1
#                                        elif self.cross_detect == 1 and self.loss_line_temp_4 == 1:
#                                            #print("okokok")
#                                            pass
#                                        else:
#                                            self.loss_line_temp_4 = 0
                                        if self.temp_1 == 1 :
                                            self.flag_2 = 1
                                            if ((self.now_encoder) + (self.t_enc)) <= (self.time + 15)*self.encoder_var :
                                                self.temp_1 = 0
                                                self.balance_flag = 0
                                                self.turn_flag = 0
                                                self.loss_line_temp_4 = 0
                                            else:
                                                self.loss_line_temp_4 = 0
                                                self.vel_pub.publish(-1000)  #left
                                                self.ste_pub.publish(8300)
                                        elif self.temp_1 == 2:
                                            self.flag_2 = 1
                                            if ((self.now_encoder) + (self.t_enc)) <= (self.time + 15)*self.encoder_var :
                                                self.temp_1 = 0
                                                self.balance_flag = 0
                                                self.turn_flag = 0
                                                self.loss_line_temp_4 = 0
                                            else:
                                                self.loss_line_temp_4 = 0
                                                self.vel_pub.publish(-1000)  #right
                                                self.ste_pub.publish(2000)
#                                        if self.cross_detect == 1 and self.row_count == self.row :#uncomment here when done
#                                            self.stop_encoder = -(self.t_enc)
#                                            ###print "stop_encoder = ",self.stop_encoder
#                                            self.flag_2 = 1
#                                            self.take_pallet = 6
                                else:          
#                                    if self.cross_detect == 1 and self.loss_line_temp_4 == 0  :
#                                        self.row_count += 1
#                                        self.loss_line_temp_4 = 1
#                                    elif self.cross_detect == 1 and self.loss_line_temp_4 == 1:
#                                        #print("okokok")
#                                        pass                                        
#                                    else:
#                                        self.loss_line_temp_4 = 0
                                    self.flag = 0
                                    self.angle_controll(-1200)  
                                        #print("33333333333",self.loss_line_temp_4)
                                        #print("self.row_count333333333 = ",self.row_count)
                        elif self.take_pallet == 6:
                            self.flag_2 = 1
                            self.count_2 = self.count_2 + 1
                            if self.count_2 > 20:
                                self.take_pallet = 7
                                self.count_2 = 0
                                print("self.row_count",self.row_count)
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(self.home_value)
                        elif self.take_pallet == 7:
                            self.flag_2 = 1
                            if self.has_sub_line == "yes":
                                if self.pallet == 0:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    #self.lift_pub.publish("lift_up")
                                    if self.lift_val == 1:
                                        self.count_7 += 1
                                        if self.count_7 >=30:
                                            self.count_7 = 0
                                            self.lift_pub.publish("lift_stop")
                                            self.take_pallet = 9
                                            self.loss_line_temp_4 = 0
                                            self.program_pub.publish(3203)
                                    else:
                                        self.lift_pub.publish("lift_up")
                                elif self.pallet == 1:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    #self.lift_pub.publish("lift_up")
                                    if self.lift_val == 2:
                                        self.count_7 += 1
                                        if self.count_7 >=30:
                                            self.count_7 = 0
                                            self.lift_pub.publish("lift_stop")
                                            self.take_pallet = 9
                                            self.loss_line_temp_4 = 0
                                            self.program_pub.publish(3204)
                                    else:
                                        self.lift_pub.publish("lift_down")
                            else:
                                self.flag_2 = 1
                                if self.pallet == 0:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    #self.lift_pub.publish("lift_up")
                                    if self.lift_val == 1:
                                        self.count_7 += 1
                                        if self.count_7 >=30:
                                            self.count_7 = 0
                                            self.lift_pub.publish("lift_stop")
                                            self.take_pallet = 16
                                            self.program_pub.publish(3203)
                                    else:
                                        self.lift_pub.publish("lift_up")
                                elif self.pallet == 1:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    #self.lift_pub.publish("lift_up")
                                    if self.lift_val == 2:
                                        self.count_7 += 1
                                        if self.count_7 >=30:
                                            self.count_7 = 0
                                            self.lift_pub.publish("lift_stop")
                                            self.program_pub.publish(3204)
                                            self.take_pallet = 16
                                    else:
                                        self.lift_pub.publish("lift_down")
                                #self.count_8 += 1
                        elif self.take_pallet == 9:
                            if self.cross_front_detect == 1 :
                                if self.loss_line_temp_4 == 0  :
                                    self.row_countout += 1
                                    self.loss_line_temp_4 = 1
                                elif self.loss_line_temp_4 == 1:
                                    if self.row_countout == self.row :
                                        self.vel_pub.publish(1200)
                                        self.ste_pub.publish(self.home_value)
                                        self.last_encoder_2 = -(self.t_enc)
                                        self.take_pallet = 10
                                    else:
                                        pass
                            else:
                                self.loss_line_temp_4 = 0
                                self.angle_controll_front(1350)
                                
                        elif self.take_pallet == 10:
                            ##print("encoder_1212 = ",((self.last_encoder_2) + (self.t_enc)))
                            if self.dir_sub == 2:
                                if self.cross_detect == 1:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 11
                                elif ((self.last_encoder_2) + (self.t_enc)) >= 760*self.encoder_var :
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 12
                                else:
                                    self.vel_pub.publish(1200)
                                    self.ste_pub.publish(self.home_value)
                            elif self.dir_sub == 1:
                                if self.cross_detect == 1:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 11
                                elif ((self.last_encoder_2) + (self.t_enc)) >= 1400*self.encoder_var :
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 12
                                else:
                                    self.vel_pub.publish(1200)
                                    self.ste_pub.publish(self.home_value)
                        elif self.take_pallet == 11:
                            self.vel_pub.publish(-1000)
                            self.ste_pub.publish(self.home_value)
                            time.sleep(3)
                            self.take_pallet = 12
                        elif self.take_pallet == 12:
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(2000)
                            self.take_pallet = 13
                        elif self.take_pallet == 13:
                            if self.dir_sub == 2:
                                if self.count_9 >=50:
                                    self.vel_pub.publish(1200)
                                    self.ste_pub.publish(2000)
                                    self.take_pallet = 14
                                    self.count_9 = 0
                                    self.last_encoder_4 = -(self.t_enc)
                                else:
                                    self.vel_pub.publish(1000)
                                    self.ste_pub.publish(2000)
                                    self.count_9 += 1
                            elif self.dir_sub == 1:
                                if self.count_9 >=50:
                                    self.vel_pub.publish(-1200)
                                    self.ste_pub.publish(2000)
                                    self.take_pallet = 14
                                    self.count_9 = 0
                                    self.last_encoder_4 = -(self.t_enc)
                                else:
                                    self.vel_pub.publish(-1000)
                                    self.ste_pub.publish(2000)
                                    self.count_9 += 1
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(self.home_value)
                                self.take_pallet = 20
                        elif self.take_pallet == 14:
                            if self.dir_sub == 2:
                                if self.mag_left_value == 1 or ((self.last_encoder_4) + (self.t_enc)) > 1000*self.encoder_var:
                                    self.vel_pub.publish(1000)
                                    self.ste_pub.publish(2000)
                                    self.take_pallet = 15
                                    self.last_encoder_4 = 0
                                else:
                                    self.vel_pub.publish(1200)
                                    self.ste_pub.publish(2000)
                            elif self.dir_sub == 1:
                                if self.mag_right_value == 1 or ((self.last_encoder_4) + (self.t_enc)) < -1000*self.encoder_var:
                                    self.vel_pub.publish(-1000)
                                    self.ste_pub.publish(2000)
                                    self.take_pallet = 15
                                    self.last_encoder_4 = 0
                                else:
                                    self.vel_pub.publish(-1200)
                                    self.ste_pub.publish(2000)
                            else:
                                pass
                        elif self.take_pallet == 15:
                            if self.dir_sub == 2:
                                if self.count_front_magss >= 8:
                                    pass
                                elif self.position(self.mag_ss_front) >= 6 and self.position(self.mag_ss_front) <= 11 and self.no_line_flag_front == 0 :
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 16
                                    #print("self.position(self.mag_ss_front)",self.position(self.mag_ss_front))
                                else:
                                    self.vel_pub.publish(1000)
                                    self.ste_pub.publish(2000)
                            elif self.dir_sub == 1:
                                if self.position(self.mag_ss_front) >= 6 and self.position(self.mag_ss_front) <= 11  and self.no_line_flag_front == 0 :
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 16
                                else:
                                    self.vel_pub.publish(-1000)
                                    self.ste_pub.publish(2000)
                        elif self.take_pallet == 16:
                            if self.no_line_flag_front == 1:
                            #if self.mag_ss_front == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
                                self.count_4 += 1
                                if self.count_4 >= 80:
                                    self.take_pallet = 17
                                    self.count_4 = 0
                                else:
                                    if self.count_front_magss >= 1:
                                        self.take_pallet = 17
                                    else:
                                        #print("self.position(self.mag_ss)",self.position(self.mag_ss))
                                        if self.position(self.mag_ss) < 8 :#and self.count_magss > 10:
                                            self.vel_pub.publish(1200)
                                            self.ste_pub.publish(self.home_value + 100)
                                        elif self.position(self.mag_ss) >= 8 :#and self.count_magss > 10:
                                            self.vel_pub.publish(1200)
                                            self.ste_pub.publish(self.home_value - 100)
                                        else:
                                            pass
                            else:
                                self.take_pallet = 17
                        elif self.take_pallet == 17:
                            self.count_12 += 1
                            if self.count_12 > 100:
                                self.take_pallet = 18
                                self.count_12 = 0
                            else:
                                self.count_10 = 0
                                self.angle_controll_front(1250)
                        elif self.take_pallet == 18:
                            if self.mag_ss_front == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
                                if self.line_ord == 3:
                                    self.take_pallet = 19
                                else:
                                    self.count_10 += 1
                                    if self.count_10 >= 15:
                                        self.take_pallet = 19
                                        self.count_10 = 0
                            else:
                                self.count_12 = 0
                                self.angle_controll_front(1650)
                        elif self.take_pallet == 19:
                            if self.line_ord == 3:
                                self.count_10 += 1
                                if self.count_10 >= 20:
                                    if self.has_sub_line == "yes":
                                        self.vel_pub.publish(0)
                                        self.ste_pub.publish(self.home_value)
                                        self.take_pallet = 20
                                    else:
                                        self.vel_pub.publish(0)
                                        self.ste_pub.publish(self.home_value)
                                        self.take_pallet = 20
                                else:
                                    self.angle_controll_front(1200)
                            else:
                                if self.has_sub_line == "yes":
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 20
                                else:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 20
                            
                        elif self.take_pallet == 20:
                            self.flag_2 = 1
                            self.count_11 += 1
                            if self.count_11 < 10:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(self.home_value)
                            elif self.count_11 > 10 and self.count_11 < 30:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(2000)
                            elif self.count_11 > 30:
                                self.last_encoder_1 = -(self.t_enc)
                                self.take_pallet = 21
                                self.count_11 = 0
                            else:
                                pass
                        elif self.take_pallet == 21:
                            if self.dir_out == 3:
                                self.flag_2 = 1
                                ###print("encoder_here",((self.last_encoder_1) + (self.t_enc)))
                                if ((self.last_encoder_1) + (self.t_enc)) < -1700*self.encoder_var:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 22
                                else:
                                    self.vel_pub.publish(-1300)
                                    self.ste_pub.publish(2000)
                            elif self.dir_out == 2:
                                self.flag_2 = 1
                                ###print("encoder_here",((self.last_encoder_1) + (self.t_enc)))
                                if ((self.last_encoder_1) + (self.t_enc)) > 1700*self.encoder_var:
                                    self.vel_pub.publish(0)
                                    self.ste_pub.publish(self.home_value)
                                    self.take_pallet = 22
                                else:
                                    self.vel_pub.publish(1300)
                                    self.ste_pub.publish(2000)
                            elif self.dir_out == 1:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(self.home_value)
                                self.take_pallet = 22
                        elif self.take_pallet == 22:
                            if self.pallet == 0:
                                self.flag_2 = 1
                                self.status = 1
                                self.PID_enable = 2
                                self.take_pallet = 23
                            elif self.pallet == 1 :
                                self.flag_2 = 1
                                self.status = 2
                                self.PID_enable = 2
                                self.take_pallet = 23
                            else:
                                self.take_pallet = 23
                    elif self.temp_1 == 1  and self.flag == 1  and self.stop_flag == 0:
                        if ((self.now_encoder) + (self.t_enc)) <= self.time*self.encoder_var :
                            self.stop_flag = 1
                            self.temp_1 = 0
                            self.loss_line_flag_1 = 0
                            if self.has_sub_line == "no":
                                self.row_count = self.bay_count
                                #print("here222222222",self.row_count)
                            else:
                                pass
                        else:
                            self.vel_pub.publish(-1000)  #left
                            self.ste_pub.publish(8300)
                    elif self.temp_1 == 2 and self.flag == 2 and self.stop_flag == 0:
                        if ((self.now_encoder) + (self.t_enc)) <= self.time*self.encoder_var :
                            self.stop_flag = 1
                            self.temp_1 = 0
                            self.loss_line_flag_1 = 0
                            if self.has_sub_line == "no":
                                self.row_count = self.bay_count
                                #print("here222222222",self.row_count)
                            else:
                                pass

                        else:
                            self.vel_pub.publish(-1000)  #right
                            self.ste_pub.publish(2000)
                    elif self.stop_flag == 0 and self.pos_left == 1:
                        if self.cross_detect == 1 :
                            if self.loss_line_temp_3 == 0  :
                                self.bay_count += 1
                                self.loss_line_temp_3 = 1
                            elif self.loss_line_temp_3 == 1:
                                if self.has_sub_line == "yes" and self.bay_count == self.bay:
                                #and self.bay_count == self.bay :#uncomment here when done
                                    self.now_encoder = -(self.t_enc)
                                    if self.bay_count == self.bay:
                                        self.take_pallet = 2
                                        self.temp_1 = 1
                                        self.flag = 1
                                        self.turn_flag = 0
                                        #print("here1111111112222",self.bay_count)
                                    else:
                                        self.take_pallet = 1
                                        self.temp_1 = 1
                                        self.flag = 1
                                        self.turn_flag = 0
                                        #self.row_count = self.bay_count
                                        #print("here111111111")
                                elif self.has_sub_line == "no" and self.bay_count == self.row:
                                    #and self.bay_count == self.row :#uncomment here when done
                                    self.now_encoder = -(self.t_enc)
                                    if self.bay_count == self.row:
                                        self.flag_2 = 1
                                        self.take_pallet = 6
                                        self.temp_1 = 1
                                        self.flag = 1
                                        self.turn_flag = 0
                                        #print("here1111111112222",self.bay_count)
                                    else:
                                        self.take_pallet = 5
                                        self.temp_1 = 1
                                        self.flag = 1
                                        self.turn_flag = 0
                                        self.row_count = self.bay_count
                                else:
                                    pass
                        else:
                            self.loss_line_temp_3 = 0
                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                ##print "self.temp = ",self.temp
                                self.temp_1 = 1
                                self.flag = 1
                                self.vel_pub.publish(-1000)#21h-11/12/2017
                                self.ste_pub.publish(self.home_value)
                                self.now_encoder = -(self.t_enc)
                            else:
                                self.angle_controll(-1200)
                    elif self.stop_flag == 0 and self.pos_right == 1:
                        if self.cross_detect == 1 :
                            if self.loss_line_temp_3 == 0  :
                                self.bay_count += 1
                                self.loss_line_temp_3 = 1
                            elif self.loss_line_temp_3 == 1:
                                if self.has_sub_line == "yes" and self.bay_count == self.bay:
                                    self.now_encoder = -(self.t_enc)
                                    if self.bay_count == self.bay:
                                        self.take_pallet = 2
                                        self.temp_1 = 2
                                        self.flag = 2
                                        #print("here1111111112222",self.bay_count)
                                    else:
                                        self.take_pallet = 1
                                        self.temp_1 = 2
                                        self.flag = 2
                                        self.turn_flag = 0
                                        #self.row_count = self.bay_count
                                        #print("here111111111")
                                elif self.has_sub_line == "no" and self.bay_count == self.row:
                                    self.now_encoder = -(self.t_enc)
                                    if self.bay_count == self.row:
                                        self.flag_2 = 1
                                        self.temp_1 = 2
                                        self.flag = 2
                                        self.take_pallet = 6
                                        #print("here1111111112222",self.bay_count)
                                    else:
                                        self.take_pallet = 5
                                        self.temp_1 = 2
                                        self.flag = 2
                                        self.turn_flag = 0
                                        self.row_count = self.bay_count
                                else:
                                    pass
                        else:
                            self.loss_line_temp_3 = 0
                            if pos == 8 and self.count_magss < 8 and self.line_flag == 0:
                                self.vel_pub.publish(-1000)
                                self.ste_pub.publish(self.home_value)
                                self.temp_1 = 2
                                self.flag = 2
                                self.now_encoder = -(self.t_enc)
                            else: 
                                self.angle_controll(-1200)
                    else:
                        self.stop_flag = 1
        else:
            self.count_lane = 2
            self.loss_line_flag = 1
            self.loss_line_temp_2 = 1
        
#            #####################__moving_charger__#####################
    def moving_charger(self):
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
                self.count_lane = 1
        elif self.count_lane == 1 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0:
            self.loss_line_charger()
        elif self.count_lane == 2 and self.loss_line_flag == 1 and self.loss_line_temp_2 == 1:
            if self.mag_flag(self.mag_ss_front) == 1 and self.flag_2 == 0:
                self.vel_pub.publish(0)
                self.ste_pub.publish(5200)
                self.count_3 += 1
                if self.count_3 > 30:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(5200)
                    self.error_pub.publish(3215)
                    self.PID_enable = 2
                    self.status = 3
                    self.count_3 = 0
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(self.home_value)
                ##print "Can not find lane to run "
                
            else:
                #pos = self.position(self.mag_ss)
                if self.take_pallet == 0:
                    self.take_pallet = 1
#                    self.take_pallet = 1
#                    if self.lift_val == 1 or self.lift_val == 0:
#                        self.vel_pub.publish(0)
#                        self.ste_pub.publish(5200)
#                        self.lift_pub.publish("lift_down")
#                        if self.lift_val == 2:
#                            self.lift_pub.publish("lift_stop")  
#                            self.take_pallet = 1
#                    else:
#                        self.take_pallet = 1
                elif self.take_pallet == 1:
                    if self.count_front_magss == 0:
                        self.now_encoder = -(self.t_enc)
                        self.take_pallet = 2
                        self.flag_2 = 1
                    else:
                        if self.cross_front_detect == 1 :#uncomment here when done
                            self.stop_encoder = -(self.t_enc)
                            self.take_pallet = 6
                        else:
                            self.flag = 0
                            self.angle_controll_front(1200)
                elif self.take_pallet == 2:
                    self.flag_2 = 1
                    if ((self.now_encoder) + (self.t_enc)) > 500*self.encoder_var:
                        for i in range(30):
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(self.home_value)
                        self.take_pallet = 3
                    else:
                        self.vel_pub.publish(1100)
                        self.ste_pub.publish(self.home_value)
                elif self.take_pallet == 3:
                    self.flag_2 = 1
                    for i in range(30):
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(2000)
                        self.take_pallet = 4
                elif self.take_pallet == 4:
                    self.flag_2 = 1
                    if self.position(self.mag_ss_front) < 16 and self.no_line_flag_front == 0 :
                        self.take_pallet = 5
                        self.flag_2 = 0
                    else:
                        self.vel_pub.publish(1100)
                        self.ste_pub.publish(2000)
                elif self.take_pallet == 5:
                    if self.cross_front_detect == 1 :#uncomment here when done
                        self.stop_encoder = -(self.t_enc)
                        self.take_pallet = 6
                    else:
                        self.flag_2 = 0
                        self.flag = 0
                        self.angle_controll_front(1200)  
                        ##print("self.bay_count = ",self.bay_count)
#                        if self.count_front_magss < 16 and self.loss_line_temp_3 == 0  :
#                            self.bay_count += 1
#                            self.loss_line_temp_3 = 1
#                        if self.loss_line_temp_3 == 1:
#                            if self.cross_detect == 1:
#                                pass
#                            else:
#                                self.loss_line_temp_3 = 0
                elif self.take_pallet == 6:
                    self.count_2 = self.count_2 + 1
                    if self.count_2 > 30:
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
                        self.take_pallet = 7
                        self.program_pub.publish(3205)
                        self.PID_enable = 2
                        self.count_2 = 0
                        self.charger_flag = 1
                    else:
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(self.home_value)
        else :
            self.count_lane = 2
            self.loss_line_flag = 1
            self.loss_line_temp_2 = 1
            
    def turn_charger(self) :
        if self.take_pallet ==0:
            for i in range(30):
                self.vel_pub.publish(0)
                self.ste_pub.publish(2000)
            self.take_pallet = 1
            self.last_encoder_1 = -(self.t_enc)
        elif self.take_pallet == 1:
            if self.mag_right_value == 0:
                self.take_pallet = 2
                print("self.encoder",((self.last_encoder_1) + (self.t_enc)))
            else:
                self.vel_pub.publish(-1100)
                self.ste_pub.publish(2000)
        elif self.take_pallet == 2:
            for i in range(30):
                self.vel_pub.publish(0)
                self.ste_pub.publish(2000)
            self.program_pub.publish(3206)
            self.PID_enable = 2
            self.count_2 = 0
            self.charger_flag = 1
    def start_charger(self):
            self.charge_cmd.publish(5000)

        
        #####################__moving_out_charger__#####################
    def return_ready(self):
        if self.temp_1 == 0:
            self.last_encoder_1 = -(self.t_enc)
            self.temp_1 = 1
#            self.reset_run_pub.publish(2000)
        elif self.temp_1 == 1:
            if ((self.last_encoder_1) + (self.t_enc)) > 270*self.encoder_var:
                self.temp_1 = 2
                self.PID_enable = 2
                self.charger_flag = 1
                self.program_pub.publish(3207)
                self.last_encoder_1 = -(self.t_enc)
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
            else:
                self.vel_pub.publish(1100)
                self.ste_pub.publish(2000)     
        
    def moving_out_charger(self):
        if self.temp_1 == 0:
            self.last_encoder_1 = -(self.t_enc)
            self.temp_1 = 2
#            self.reset_run_pub.publish(2000)
#        elif self.temp_1 == 1:
#            if ((self.last_encoder_1) + (self.t_enc)) > 270:
#                self.temp_1 = 2
#                self.last_encoder_1 = -(self.t_enc)
#                self.vel_pub.publish(0)
#                self.ste_pub.publish(self.home_value)
#            else:
#                self.vel_pub.publish(1100)
#                self.ste_pub.publish(2000) 
        elif self.temp_1 == 2:
            if ((self.last_encoder_1) + (self.t_enc)) < -3000*self.encoder_var:
                self.temp_1 = 3
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
            else:
                self.vel_pub.publish(-1200)
                self.ste_pub.publish(5200) 
        elif self.temp_1 == 3:
            for i in range(30):
                self.vel_pub.publish(0)
                self.ste_pub.publish(self.home_value)
            self.last_encoder_1 = -(self.t_enc)
            self.temp_1 = 4
        elif self.temp_1 == 4:
            for i in range(30):
                self.vel_pub.publish(0)
                self.ste_pub.publish(2000)
            self.temp_1 = 5
        elif self.temp_1 == 5:
            if ((self.last_encoder_1) + (self.t_enc)) > 1000*self.encoder_var:
                for i in range(30):
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(self.home_value)
                self.temp_1 = 6
            else:
                self.vel_pub.publish(1300)
                self.ste_pub.publish(2000)
        elif self.temp_1 == 6:
            for i in range(30):
                self.vel_pub.publish(0)
                self.ste_pub.publish(5200)
            self.program_pub.publish(3213)
            self.temp_enc = 0
            self.PID_enable = 2
            self.charger_flag = 0
        
    ################################__MAIN__###################################
    def main(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.PID_enable == 2:
                if self.status == 1:
                    self.program_pub.publish(3213)
                elif self.status == 2:
                    self.program_pub.publish(3213)
                else:
                    pass
                #self.s.close()
                self.vel_pub.publish(0)
                self.ste_pub.publish(5200)
                self.laser_data             = []
                self.PID_enable             = 0
                self.lift_val               = 3
                self.stop_flag              = 0
                self.stop_encoder           = 0
                self.stop_flag_laser        = 0
                self.flag                   = 0
                self.flag_1                 = 0
                self.flag_laser             = 0
                self.mag_ss                 = []
                self.error                  = 0
                self.set_point              = 8
                self.last_proportional      = 0
                self.last_ster_val          = 0
                self.integral               = 0
                self.home_value             = 5200
                self.angle                  = 0
                self.temp                   = 0
                self.pos_left               = 0
                self.pos_right              = 0
                self.pos_stop               = 0
                self.center                 = 1
                self.list_a                 = [7,6,5]
                self.list_b                 = [10,11,12]
                self.stear_enc              = 0
                self.t_enc                  = 0
                self.temp_1                 = 0
                self.time                   = 0
                self.cross_detect           = 0
                self.take_pallet            = 0
                self.status                 = 0
                self.temp_2                 = 0
                self.full_line_flag         = 0
                self.count                  = 0
                self.count_1                = 0
                self.count_2                = 0
                self.count_3                = 0
                self.count_4                = 0
                self.count_5                = 0
                self.count_6                = 0
                self.count_7                = 0
                self.count_8                = 0
                self.count_9                = 0
                self.count_11               = 0
                self.count_12               = 0
                self.now_encoder            = 0
                self.last_encoder           = 0
                self.last_encoder_1         = 0
                self.last_encoder_2         = 0
                self.last_encoder_3         = 0
                self.last_encoder_4         = 0
                self.loss_line_flag         = 0
                self.loss_line_flag_1       = 0
                self.loss_line_temp         = 0
                self.loss_line_stt          = 0
                self.loss_line_temp_1       = 0
                self.loss_line_temp_2       = 0
                self.loss_line_temp_3       = 0
                self.loss_line_temp_4       = 0
                self.out_take_pallet        = 0
                self.out_put_pallet         = 0
                self.loss_line_temp_6       = 0
                self.count_lane             = 0
                self.count_magss            = 0
                self.mag_add_flag           = 3
                self.line_flag              = 0
                self.vel_pub.publish(0)
                self.ste_pub.publish(5200)
                self.PID_enable             = 0
                self.balance_flag           = 0
                self.mag_left_value         = 2
                self.mag_right_value        = 2
                self.flag_2                 = 0
                self.bay_count              = 0
                self.row_count              = 0
                self.row_countout           = 0
                self.turn_off_pc_flag       = 0
                self.no_line_flag           = 0
                self.dir_main_temp          = 0
                self.dir_main_count         = 0
                self.traffic_flag           = 2
                #self.row = 2
                self.bay                    = 0
                self.has_sub_line           = None
                self.dir_sub                = None
                self.dir_main               = None
                self.line_ord               = 0
                self.line_ord_count         = 0
                self.row                    = 0
                self.dir_out                = 0
                self.count_10               = 0 
                self.charger_ready          = 0
                self.encoder_var            = 2.2
#                self.file_count += 1
#                self.logger = logging.getLogger('line_folow_%s'%self.file_count)
#                self.hdlr = logging.FileHandler('log_line/log_%s.txt' %self.file_count)
#                self.formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
#                self.hdlr.setFormatter(self.formatter)
#                self.logger.addHandler(self.hdlr) 
#                self.logger.setLevel(logging.INFO)
                if self.charger_flag == 0:
                    self.line_pub.publish(3000)
                    #print(self.PID_enable)
                else:
                    pass
                
            elif self.PID_enable == 1:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.taking_pallet()
                        if self.cross_detect == 1:
                            print(self.mag_ss , "row",self.row_count ,"bay",self.bay_count)
                        else:
                            pass
#                        self.logger.info('mag_back ',self.mag_ss)
#                        self.logger.info('take_pallet ',self.take_pallet)
#                        self.logger.info('row_count ',self.row_count)
#                        self.logger.info('bay_count ',self.bay_count)
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass
            elif self.PID_enable == 3:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.moving_charger()
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass
            elif self.PID_enable == 4:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.moving_out_charger()
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass        
            elif self.PID_enable == 5:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.start_charger()
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass           
            elif self.PID_enable == 6:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.turn_charger()
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass                    
            elif self.PID_enable == 7:
                if self.flag_laser == 0:
                    self.line_pub.publish(1000)
                    self.flag_laser = 1
                elif self.flag_laser == 1:
                    if self.count_8 > 50:
                        self.flag_laser = 2
                        self.count_8 = 0
                    else:
                        self.count_8 += 1
                elif self.flag_laser == 2:
                    if self.traffic_flag == 2:
                        self.return_ready()
                    elif self.traffic_flag == 0:
                        self.vel_pub.publish(0)
                    else:
                        pass                  
            elif self.PID_enable == 0:  
                pass
                ##print ' Waiting... '
            r.sleep()
if __name__ == '__main__':
    run = line_follow()
    run.main()
