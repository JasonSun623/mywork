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
        self.home_value = 550
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
        self.now_encoder = 0
        self.last_encoder = 0
        self.last_encoder_1 = 0
        self.last_encoder_2 = 0
        self.loss_line_flag = 0
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
    
        #####################__Timer__#####################
    def timer(self,pos):
        if pos >= 4 and pos < 6:
            self.time = -330#0.9
        elif pos >= 6 and pos < 7:
            self.time = -200#0.6
        elif pos < 4 :
            self.time = -570#1.4
        elif pos > 9 and pos <= 12:
            self.time = -200#0.6
        elif pos > 12 and pos <= 14:
            self.time = -330#0.9
        elif pos > 14:
            self.time = -570#1.4
        self.line_pos.publish(pos)
        print "pos_1 = ",pos
        
        
        
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
        ster_value = int(proportional* kp +  derivative*kd) #- 1
        self.last_proportional = proportional
        #print "pos = ",pos,"ster_value = ",ster_value
        return ster_value
        
    #####################__angle_control__#####################
    def angle_controll(self,speed):
        pos = self.position(self.mag_ss)
        turning_value = self.pid_cal(self.position(self.mag_ss),15,200)
        #print turning_value,int(turning_value)
        self.angle = self.home_value + turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        #print "self.angle",self.angle
        if self.angle > 950:
            self.angle = 950
        elif self.angle < 5:
            self.angle = 5
        print "speed",speed, "angle",self.angle
        #print "left = ", self.left,"right = ",self.right,"center = ",self.center
        self.vel_pub.publish(speed)
        self.ste_pub.publish(self.angle)
    #####################__angle_control_charger__#####################
    def angle_controll_charger(self,speed):
        pos = self.position(self.mag_ss)
        turning_value = self.pid_cal(self.position(self.mag_ss),15,200)
        #print turning_value,int(turning_value)
        self.angle = self.home_value + turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        #print "self.angle",self.angle
        if self.angle > 950:
            self.angle = 950
        elif self.angle < 5:
            self.angle = 5
        print "speed",speed, "angle",self.angle
        #print "left = ", self.left,"right = ",self.right,"center = ",self.center
        self.vel_pub.publish(speed)
        self.ste_pub.publish(self.angle)
        #####################__Loss_line_left__#####################
    def loss_line(self):
        if self.loss_line_temp == 0:
            self.last_encoder = -(self.t_enc)
            self.loss_line_temp = 1
        elif self.loss_line_temp == 1:
            if ((self.last_encoder) + (self.t_enc)) <= -2500 :
                self.loss_line_stt
                if self.loss_line_stt == 2:
                    print self.loss_line_stt
                    if self.loss_line_temp_4 == 0:
                        print self.mag_ss
                        if self.mag_ss == [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] or self.mag_ss == [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] or self.mag_ss == [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] :
                            print"((self.last_encoder_2) + (self.t_enc)) = ",((self.last_encoder_2) + (self.t_enc))
                            if ((self.last_encoder_2) + (self.t_enc)) <= -1200:
                                self.vel_pub.publish(10)
                                self.ste_pub.publish(550)
                                self.loss_line_temp_4 = 1
                            else:
                                print "hereeeeeeee"
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                                #self.vel_pub.publish(-500)
                                #self.ste_pub.publish(550)
                                
                        elif self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] or self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1] or self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1]:
                            print"((self.last_encoder_2) + (self.t_enc)) = ",((self.last_encoder_2) + (self.t_enc))
                            if ((self.last_encoder_2) + (self.t_enc)) <= -1200:
                                self.vel_pub.publish(100)
                                self.ste_pub.publish(550)
                                self.loss_line_temp_4 = 2
                            else:
                                print "hereeeeeeee"
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                                #self.vel_pub.publish(-500)
                                #self.ste_pub.publish(550)
                                
                        else:
                            if self.loss_line_temp_5 == 0:
                                self.last_encoder_2 =-(self.t_enc)
                                self.vel_pub.publish(-500)
                                self.ste_pub.publish(650)
                                self.loss_line_temp_5 = 1
                            elif self.loss_line_temp_5 == 1:
                                #self.last_encoder_2 =-(self.t_enc)
                                self.vel_pub.publish(-500)
                                self.ste_pub.publish(650)
                                self.loss_line_temp_5 = 1
                    elif self.loss_line_temp_4 == 1:
                        if self.loss_line_temp_3 == 0:
                            self.last_encoder_1 = -(self.t_enc)
                            self.loss_line_temp_3 = 1
                        elif self.loss_line_temp_3 == 1:
                            print "encoder = ",((self.last_encoder_1) + (self.t_enc))
                            if ((self.last_encoder_1) + (self.t_enc)) <= -110 :
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                            else:
                                self.vel_pub.publish(-600)
                                self.ste_pub.publish(800)
                    elif self.loss_line_temp_4 == 2 :
                        if self.loss_line_temp_3 == 0:
                            self.last_encoder_1 = -(self.t_enc)
                            self.loss_line_temp_3 = 1
                        elif self.loss_line_temp_3 == 1:
                            print "encoder = ",((self.last_encoder_1) + (self.t_enc))
                            if ((self.last_encoder_1) + (self.t_enc)) <= -110 :
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                            else:
                                self.vel_pub.publish(-600)
                                self.ste_pub.publish(300)
                elif self.loss_line_stt == 1:    
                    print self.loss_line_stt
                    if self.loss_line_temp_4 == 0:
                        print self.mag_ss
                        if self.mag_ss == [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] or self.mag_ss == [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] or self.mag_ss == [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] :
                            print"((self.last_encoder_2) + (self.t_enc)) = ",((self.last_encoder_2) + (self.t_enc))
                            if ((self.last_encoder_2) + (self.t_enc)) <= -1200:
                                self.vel_pub.publish(10)
                                self.ste_pub.publish(550)
                                self.loss_line_temp_4 = 1
                            else:
                                print "hereeeeeeee"
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                                #self.vel_pub.publish(-500)
                                #self.ste_pub.publish(550)
                                
                        elif self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] or self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1] or self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1]:
                            print"((self.last_encoder_2) + (self.t_enc)) = ",((self.last_encoder_2) + (self.t_enc))
                            if ((self.last_encoder_2) + (self.t_enc)) <= -1200:
                                self.vel_pub.publish(10)
                                self.ste_pub.publish(550)
                                self.loss_line_temp_4 = 2
                            else:
                                print "hereeeeeeee"
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                                #self.vel_pub.publish(-500)
                                #self.ste_pub.publish(550)
                        else:
                            if self.loss_line_temp_5 == 0:
                                self.last_encoder_2 =-(self.t_enc)
                                self.vel_pub.publish(-500)
                                self.ste_pub.publish(450)
                                self.loss_line_temp_5 = 1
                            elif self.loss_line_temp_5 == 1:
                                #self.last_encoder_2 =-(self.t_enc)
                                self.vel_pub.publish(-500)
                                self.ste_pub.publish(450)
                    elif self.loss_line_temp_4 == 1:
                        if self.loss_line_temp_3 == 0:
                            self.last_encoder_1 = -(self.t_enc)
                            self.loss_line_temp_3 = 1
                        elif self.loss_line_temp_3 == 1:
                            print "encoder = ",((self.last_encoder_1) + (self.t_enc))
                            if ((self.last_encoder_1) + (self.t_enc)) <= -110 :
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                            else:
                                self.vel_pub.publish(-550)
                                self.ste_pub.publish(800)
                    elif self.loss_line_temp_4 == 2 :
                        if self.loss_line_temp_3 == 0:
                            self.last_encoder_1 = -(self.t_enc)
                            self.loss_line_temp_3 = 1
                        elif self.loss_line_temp_3 == 1:
                            print "encoder = ",((self.last_encoder_1) + (self.t_enc))
                            if ((self.last_encoder_1) + (self.t_enc)) <= -110 :
                                self.loss_line_flag = 1
                                self.loss_line_temp_2 = 1
                                self.count_lane = 2
                                self.loss_line_temp_3 = 0
                            else:
                                self.vel_pub.publish(-550)
                                self.ste_pub.publish(300)
                elif self.loss_line_stt == 3:
                    self.loss_line_flag = 1
                    self.loss_line_temp_2 = 1
                    self.count_lane = 2
                    self.loss_line_temp_3 = 0
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                    self.count_3 += 1
                    if self.count_3 > 30:
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(550)
                        self.error_pub.publish(4205)
                        self.PID_enable = 2
                        self.status = 3
                        
                    else:
                        self.vel_pub.publish(0)
                        self.ste_pub.publish(550)
                    print "still not detect the lane"
            else:
                self.vel_pub.publish(-500)
                self.ste_pub.publish(550)
                if self.count_magss > 3 :
                    print self.count_magss
                    if self.count_magss > 3 and self.loss_line_temp_1 == 0 :
                        self.loss_line_stt += 1
                        self.loss_line_temp_6 +=1
                        self.loss_line_temp_1 = 1
                    elif self.count_magss > 3 and self.loss_line_temp_1 == 1:
                        self.loss_line_temp_6 +=1
                        print "self.loss_line_temp_6 = ",self.loss_line_temp_6
                        if self.loss_line_temp_6 >= 30:
                            self.loss_line_stt = 3
                        
                    else:
                        pass
                else :
                    self.loss_line_temp_1 = 0
                    
        #####################__taking_pallet__#####################
    def taking_pallet(self):
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
                self.count_lane = 1
        elif self.count_lane == 1 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0:
            self.loss_line()
        elif self.count_lane == 2 and self.loss_line_flag == 1 and self.loss_line_temp_2 == 1:
            if self.mag_flag(self.mag_ss) == 1:
                self.vel_pub.publish(0)
                self.ste_pub.publish(550)
                self.count_3 += 1
                if self.count_3 > 30:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                    self.error_pub.publish(4205)
                    self.PID_enable = 2
                    self.status = 3
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                print "Can not find lane to run "
                
            else:
                if self.temp_2 == 0:
                    self.count += 1
                    if self.count >= 20:
                        self.temp_2 = 1
                        self.count = 0
                    else:
                        pos = self.position(self.mag_ss)
                        self.timer(pos)
                    print "self.time = ",self.time
                else:
                    #self.flag_1 = 1
                    pos = self.position(self.mag_ss)
                    if self.stop_flag == 1:
                        if self.take_pallet == 0:
                            #self.take_pallet = 1
                            ##########uncomment here when release#######
                            if self.lift_val == 1 or self.lift_val == 0:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.lift_pub.publish("lift_down")
                                if self.lift_val == 2:
                                    self.lift_pub.publish("lift_stop")  
                                    self.take_pallet = 1
                            else:
                                self.take_pallet = 1
                            
                        elif self.take_pallet == 1:
                            if self.cross_detect == 1 and self.pos_stop == 1 :
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.take_pallet = 2
                                self.pos_stop = 0
                            else:
                                self.flag = 0
                                self.angle_controll(-1150)    
                        
                        elif self.take_pallet == 2:
                            self.count_2 = self.count_2 + 1
                            if self.count_2 > 20:
                                self.take_pallet = 3
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                        elif self.take_pallet == 3:
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(550)
                            self.lift_pub.publish("lift_up")
                            if self.lift_val == 1:
                                self.lift_pub.publish("lift_stop")
                                self.take_pallet = 4
                        elif self.take_pallet == 4:
                            self.status = 1
                            self.PID_enable = 2
                            self.take_pallet = 5
                    elif self.temp_1 == 1  and self.flag == 1 :# and self.stop_flag == 0:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(800)
                        """
                        for i in range(10):
                            self.vel_pub.publish(-650)
                            self.ste_pub.publish(800)
                        print('Before: %s' ,self.t_enc)
                        time.sleep(self.time)
                        print(self.time)
                        print('After: %s\n' ,self.t_enc)
                        self.stop_flag = 1
                        self.temp_1 = 0
                        #"""
                    elif self.temp_1 == 2 and self.flag == 2: #and self.stop_flag == 0:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(300)
                        """
                        for i in range(10):
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(300)
                        print('Before: %s',self.t_enc)
                        print(self.time)
                        time.sleep(self.time)
                        print('After: %s\n', self.t_enc)
                        self.stop_flag = 1
                        self.temp_1 = 0
                        """
                    elif self.stop_flag == 0 and self.pos_left == 1:
                        print " move from the left",self.count_magss
                        if pos == 8 and self.count_magss < 8:
                            print "self.temp = ",self.temp
                            self.temp_1 = 1
                            self.flag = 1
                            self.vel_pub.publish(-500)#21h-11/12/2017
                            self.ste_pub.publish(550)
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    elif self.stop_flag == 0 and self.pos_right == 1:
                        print "move from the right",self.count_magss
                        if pos == 8 and self.count_magss < 8 :
                            self.vel_pub.publish(-500)
                            self.ste_pub.publish(550)
                            self.temp_1 = 2
                            self.flag = 2
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    else:
                        self.stop_flag = 1
        else:
            self.count_lane = 2
            self.loss_line_flag = 1
            self.loss_line_temp_2 = 1
        #####################__putdown_pallet__#####################
    def put_down_pallet(self):
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
                self.count_lane = 1
        elif self.count_lane == 1 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0:
            self.loss_line()
        elif self.count_lane == 2 and self.loss_line_flag == 1 and self.loss_line_temp_2 == 1:
            if self.mag_flag(self.mag_ss) == 1:
                self.vel_pub.publish(0)
                self.ste_pub.publish(550)
                self.count_3 += 1
                if self.count_3 > 30:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                    self.error_pub.publish(4205)
                    self.PID_enable = 2
                    self.status = 3
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                print "Can not find lane to run "
            else:
                if self.temp_2 == 0:
                    self.count += 1
                    if self.count >= 20:
                        self.temp_2 = 1
                        self.count = 0
                    else:
                        pos = self.position(self.mag_ss)
                        self.timer(pos)
                    print "self.timer = ",self.time
                else:
                    self.flag_1 = 1
                    pos = self.position(self.mag_ss)
                    if self.stop_flag == 1:
                        if self.take_pallet == 0:
                            if self.cross_detect == 1 and self.pos_stop == 1 :
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.take_pallet = 1
                                self.pos_stop = 0
                            else:
                                self.flag = 0
                                self.angle_controll(-1150)                
                        elif self.take_pallet == 1:
                            self.count_2 = self.count_2 + 1
                            if self.count_2 > 20:
                                self.take_pallet = 2
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                        elif self.take_pallet == 2:
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(550)
                            self.lift_pub.publish("lift_down")
                            if self.lift_val == 2:
                                self.lift_pub.publish("lift_stop")
                                self.take_pallet = 3
                        elif self.take_pallet == 3:
                            self.status = 2
                            self.PID_enable = 2
                            self.take_pallet = 4
                    elif self.temp_1 == 1 or self.flag == 1:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(800)
                        """
                        for i in range(10):
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(800)
                        print('Before: %s' ,self.t_enc)
                        time.sleep(self.time)
                        print(self.time)
                        print('After: %s\n' ,self.t_enc)
                        self.stop_flag = 1
                        self.temp_1 = 0
                        """
                    elif self.temp_1 == 2 or self.flag == 2:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(300)
                        """
                        for i in range(10):
                            self.vel_pub.publish(-650)
                            self.ste_pub.publish(300)
                        print('Before: %s' ,self.t_enc)
                        time.sleep(self.time)
                        print(self.time)
                        print('After: %s\n',self.t_enc)
                        self.stop_flag = 1
                        self.temp_1 = 0
                        """
                    elif self.stop_flag == 0 and self.pos_left == 1:
                        print " move from the left",self.count_magss
                        if pos == 8 and self.count_magss < 8:
                            self.temp_1 = 1
                            self.flag = 1
                            self.vel_pub.publish(-500)#21h-11/12/2017
                            self.ste_pub.publish(550)
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    elif self.pos_right == 1 and self.stop_flag == 0:
                        print "move from the right",self.count_magss
                        
                        if pos == 8 and self.count_magss < 8:
                            self.vel_pub.publish(-500)
                            self.ste_pub.publish(550)
                            self.temp_1 = 2
                            self.flag = 2
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    else:
                        self.stop_flag = 1
        else:
            #self.stop_flag = 1
            self.count_lane = 2
            self.loss_line_flag = 1
            self.loss_line_temp_2 = 1
            #####################__taking_pallet__#####################
    def moving_charger(self):
        if self.mag_ss == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and self.count_lane == 0 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0  :
                self.count_lane = 1
        elif self.count_lane == 1 and self.loss_line_flag == 0 and self.loss_line_temp_2 == 0:
            self.loss_line()
        elif self.count_lane == 2 and self.loss_line_flag == 1 and self.loss_line_temp_2 == 1:
            if self.mag_flag(self.mag_ss) == 1:
                self.vel_pub.publish(0)
                self.ste_pub.publish(550)
                self.count_3 += 1
                if self.count_3 > 30:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                    self.error_pub.publish(4205)
                    self.PID_enable = 2
                    self.status = 3
                else:
                    self.vel_pub.publish(0)
                    self.ste_pub.publish(550)
                print "Can not find lane to run "
                
            else:
                if self.temp_2 == 0:
                    self.count += 1
                    if self.count >= 20:
                        self.temp_2 = 1
                        self.count = 0
                    else:
                        pos = self.position(self.mag_ss)
                        self.timer(pos)
                    print "self.time = ",self.time
                else:
                    #self.flag_1 = 1
                    pos = self.position(self.mag_ss)
                    if self.stop_flag == 1:
                        if self.take_pallet == 0:
                            self.take_pallet = 1
                            if self.lift_val == 1 or self.lift_val == 0:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.lift_pub.publish("lift_down")
                                if self.lift_val == 2:
                                    self.lift_pub.publish("lift_stop")  
                                    self.take_pallet = 1
                            else:
                                self.take_pallet = 1
                            
                        elif self.take_pallet == 1:
                            if self.cross_detect == 1 :#and self.pos_stop == 1 :
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.take_pallet = 2
                                self.pos_stop = 0
                            else:
                                self.flag = 0
                                self.angle_controll_charger(-900)    
                        
                        elif self.take_pallet == 2:
                            self.count_2 = self.count_2 + 1
                            if self.count_2 > 20:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                                self.take_pallet = 3
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(550)
                        elif self.take_pallet == 3:
                            self.count_4 += 1
                            if self.count_4 > 20:
                                self.count_3 = self.t_enc
                                self.take_pallet = 4
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(250)
                                
                        elif self.take_pallet == 4:
                            if self.mag_add_flag == 0:
                                self.take_pallet = 5
                                print "encoder = ",self.t_enc - self.count_3
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(250)
                            else:
                                self.vel_pub.publish(600)
                                self.ste_pub.publish(250)
                        elif self.take_pallet == 5:
                            self.count_5 += 1
                            if self.count_5 > 20:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(250)
                                self.take_pallet = 6
                            else:
                                self.vel_pub.publish(0)
                                self.ste_pub.publish(250)
                        elif self.take_pallet == 6:
                            self.vel_pub.publish(0)
                            self.ste_pub.publish(550)
                        """
                        elif self.take_pallet == 4:
                            self.status = 1
                            self.PID_enable = 2
                            self.take_pallet = 5
                        """
                    elif self.temp_1 == 1  and self.flag == 1 :# and self.stop_flag == 0:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(800)
                    elif self.temp_1 == 2 and self.flag == 2: #and self.stop_flag == 0:
                        print (self.now_encoder) + (self.t_enc)
                        print "time = ",self.time
                        if ((self.now_encoder) + (self.t_enc)) <= self.time :
                            self.stop_flag = 1
                            self.temp_1 = 0
                        else:
                            self.vel_pub.publish(-600)
                            self.ste_pub.publish(300)
                    elif self.stop_flag == 0 and self.pos_left == 1:
                        print " move from the left",self.count_magss
                        if pos == 8 and self.count_magss < 8:
                            print "self.temp = ",self.temp
                            self.temp_1 = 1
                            self.flag = 1
                            self.vel_pub.publish(-500)#21h-11/12/2017
                            self.ste_pub.publish(550)
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    elif self.stop_flag == 0 and self.pos_right == 1:
                        print "move from the right",self.count_magss
                        if pos == 8 and self.count_magss < 8 :
                            self.vel_pub.publish(-500)
                            self.ste_pub.publish(550)
                            self.temp_1 = 2
                            self.flag = 2
                            self.now_encoder = -self.t_enc
                        else:
                            self.temp = self.temp + 1
                            self.angle_controll(-500)
                    else:
                        self.stop_flag = 1
        else:
            self.count_lane = 2
            self.loss_line_flag = 1
            self.loss_line_temp_2 = 1
    ################################__MAIN__###################################
    def main(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.PID_enable == 2:
                if self.status == 1:
                    self.program_pub.publish(3203)
                elif self.status == 2:
                    self.program_pub.publish(3204)
                else:
                    pass
                self.vel_pub.publish(0)
                self.ste_pub.publish(550)
                self.laser_data = []
                self.PID_enable = 0
                self.lift_val = 3
                self.stop_flag = 0
                self.stop_flag_laser = 0
                self.flag = 0
                self.flag_1 = 0
                self.flag_laser = 0
                self.mag_ss = []
                self.error = 0
                self.set_point = 8
                self.last_proportional = 0
                self.integral = 0
                self.home_value = 550
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
                self.now_encoder = 0
                self.last_encoder = 0
                self.last_encoder_1 = 0
                self.last_encoder_2 = 0
                self.loss_line_flag = 0
                self.loss_line_temp = 0
                self.loss_line_stt = 0
                self.loss_line_temp_1 = 0
                self.loss_line_temp_2 = 0
                self.loss_line_temp_3 = 0
                self.loss_line_temp_4 = 0
                self.loss_line_temp_5 = 0
                self.loss_line_temp_6 = 0
                self.count_lane = 0
                self.count_magss = 0
                self.mag_add_flag = 3
                self.vel_pub.publish(0)
                self.ste_pub.publish(550)
                self.PID_enable = 0
            elif self.PID_enable == 1:
                if self.stop_flag_laser >= 10 and self.flag_laser == 0:
                    self.vel_pub.publish(0)
                    print 'have object in front from 0 to 180 degree'
                    self.flag_laser = 1
                elif self.flag_laser == 1 :
                    if self.stop_flag < 10:
                        self.flag_laser = 0
                    else:
                        self.vel_pub.publish(0)
                        print 'have object in front from 0 to 180 degree'
                else:
                    self.moving_charger()
                    #self.taking_pallet()
            elif self.PID_enable == 3:
                if self.stop_flag_laser >= 10 and self.flag_laser == 0:
                    self.vel_pub.publish(0)
                    print 'have object in front from 0 to 180 degree'
                    self.flag_laser = 1
                elif self.flag_laser == 1 :
                    if self.stop_flag < 10:
                        self.flag_laser = 0
                    else:
                        self.vel_pub.publish(0)
                        print 'have object in front from 0 to 180 degree'
                else:
                    self.put_down_pallet()
            elif self.PID_enable == 0:
                #self.vel_pub.publish(0)
                #self.ste_pub.publish(550)
                
                print ' Waiting... '
                #self.loss_line()
            #print self.stear_enc,self.t_enc
            #print self.position(self.mag_ss)
            r.sleep()
        	#print "self.mag_ss",self.mag_ss
		#print "self.stop_flag",self.stop_flag
		#print "self.lift_val",self.lift_val
if __name__ == '__main__':
    run = line_follow()
    run.main()
