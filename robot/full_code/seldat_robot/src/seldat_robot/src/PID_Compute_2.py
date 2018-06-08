#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import UInt16,UInt8,Int8,Int32, UInt32, String,UInt8MultiArray,Float32
from geometry_msgs.msg import Quaternion,Vector3
from math import pi
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
#from opencv_line import line_follow
class Echo(object):

    ##############################____INIT____############################
    
    def __init__(self):
        self.laser_data = []
        self.PID_enable = 0
        self.rfid = 0
        self.lift_val = 3
        self.stop_flag = 0
        self.flag = 0
        self.flag_1 = 0
        self.flag_2 = 0
        self.status = 0
        self.right_ss = 0
        self.left_ss = 0
        self.ir_ss = 0
        self.back_line_ss = 0
        self.turning = 0
        self.turning_b = 0
        self.e = 0
        self.home = 2047
        self.angle = 0
        self.vel = 0
        self.right_value = 1397
        self.tt = 0
        self.vel_value = 2947
        self.left_value1 = 2497
        self.test = 0
        self.temp = 0
        self.motor_msg = Vector3()
        self.count = 0
	self.count_1 = 0
        self.rig_home = 1397
        self.cam_home = 324#151#88 
        self.cam_error = 0
        self.pallet = 0
        self.control = 0
        self.cam_stop = 0
        self.pallet_num = 0
        self.right_stop = 0
        self.server = 0
        self.encoder = 0
        self.encoder_home = -3600
        self.encoder_now = 0
        self.pos_encoder = 0
        self.encoder_value = 0
        self.position = 0
        self.stop = 0
        self.line = 0
        self.r_line = 0
        self.cx = 0
        self.turn = 0
        self.tem = 1
        self.count_temp = 0
        rospy.init_node('LINE_FOLLOWER')
        ###########################__SUBSCRIBER__##############################
        IR_sub = rospy.Subscriber('/FrontLine', String, self.IR_callback)
        IR_back_sub = rospy.Subscriber('/BackLine', String, self.back_ir_callback)
        key_sub = rospy.Subscriber('key_press', String, self.key_callback)
        rfid_sub = rospy.Subscriber('rfid',Quaternion, self.rfid_callback)
        laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        server_sub = rospy.Subscriber('linedetectionctrl', Int32, self.server_callback)
        lift_sub = rospy.Subscriber('/lift', UInt32, self.lift_callback)
        cam_sub = rospy.Subscriber('CX_value', UInt32, self.cam_callback)
        sub = rospy.Subscriber('cx_value', UInt32, self.callback)
        pallet_pos_sub = rospy.Subscriber('/pospallet',Int32 , self.pallet_pos_callback)
        control_sub = rospy.Subscriber('position_pub', String, self.control_callback)
        encoder_sub = rospy.Subscriber('/pos', Vector3, self.encoder_callback)
        stop_sub = rospy.Subscriber('status', String, self.stop_callback)
        no_line_sub = rospy.Subscriber('errorturningrobot', Int32, self.no_line_callback)
        ###########################__PUBLISH__##############################
        self.vel_pub = rospy.Publisher('motor_command',Vector3,queue_size = 100)
        self.lift_pub = rospy.Publisher('lift_controll', String,queue_size = 100)
        self.program_pub = rospy.Publisher('linedetectioncallback', Int32,queue_size = 100)
        self.led_pub = rospy.Publisher('camled', Int32,queue_size = 100)
        self.stop_cam_pub = rospy.Publisher('pause_cam', String,queue_size = 100)
        self.noline_pub = rospy.Publisher('ReachedRobotdirection', String,queue_size = 100)
    ##############################____CALL_BACK____#########################

                #######################__FRONT_IR_SENSOR__########################
    def IR_callback(self,msg):
        self.front_line_ss = msg.data
        self.line = int(msg.data[0]) + int(msg.data[1]) + int(msg.data[2]) + int(msg.data[3]) + int(msg.data[4]) + int(msg.data[5])
        self.r_line = msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3] + msg.data[4]
    def back_ir_callback(self,msg):
        self.back_line_ss = int(msg.data[9]) + int(msg.data[8]) + int(msg.data[7]) + int(msg.data[6]) + int(msg.data[5])
                #######################__TELEOP__########################
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
                #######################__TELEOP__########################            
    def encoder_callback(self,msg):
        self.encoder = msg.x
                #######################__RFID__########################
    def rfid_callback(self,msg):
	x = int(msg.x)
	y = int(msg.y)
	z = int(msg.z)
	w = int(msg.w)
	self.rfid = str(x)+str(y)+str(z)+str(w)

                #######################__LASER__########################
    def laser_callback(self,msg):
        del self.laser_data[:]
        for i in range(0,180):
                self.laser_data.append(msg.ranges[i])
        self.stop_flag = list(filter(lambda x: x > 0 and x < 0.9  ,self.laser_data))
        self.stop_flag = len(self.stop_flag)
                #######################__LASER__########################
    def server_callback(self,msg):                      
        data = msg.data
	if data == 1203:
		self.PID_enable = 1
	elif data == 1200:
		self.PID_enable = 2
		self.status = 0
	elif data == 1204:
                self.PID_enable = 3
        '''
        elif data == 1206: # out mode
                self.PID_enable = 3
        elif data == 1207: #taking pallet
                self.PID_enable = 3
        '''
    def no_line_callback(self,msg):
        data = msg.data
        if data == -1:
            self.turn = 'turn left'
        elif data == 1:
            self.turn = 'turn right'
        else:
            pass
                #######################__STOP__########################
    def stop_callback(self,msg):
        self.stop = msg.data
                #######################__LIFT__########################
    def lift_callback(self,msg):
        self.lift_val = msg.data
                #######################__CAM__########################
    def cam_callback(self,msg):
        data = msg.data        
        self.cam_error = data - self.cam_home
        #if data == 240:
        #    self.cam_error = -550
        #elif data == 306:
        #    self.cam_error = 550
        #else:
    def callback(self,msg):
        data = msg.data
        if self.tem == 1:
            if data == 1:
                self.count_1 = self.count_1 + 1
                print self.count_1
                if self.count_1 >= 20:
                    self.cx = 1 #'line'
                    self.count_1 = 0
                    self.tem = 2
            elif data == 2:
                self.count_1 = self.count_1 + 1
                print self.count_1
                if self.count_1 >= 20:
                    self.cx = 2 #'noline'
                    self.count_1 = 0
                    self.tem = 2
        else:
            if data == 1:
                self.cx = 1 #'line'
            elif data == 2:
                self.cx = 2 #'noline'
                #######################__RED_DETECT__########################
    def pallet_pos_callback(self,msg):
        self.pallet = msg.data
        if self.pallet == 1205:
            self.cam_stop = 1
        #elif self.pallet == 1206:
        #    self.cam_stop = 2
        else:
            self.cam_stop = 1
                #######################__RED_DETECT__########################
    def control_callback(self,msg):
        data = msg.data
        self.control = data
    ###########################____DEFINE_FUNCTION____#######################

        ####################__error_calculation__###################
    '''
    def error(self):
	front_ss = self.front_line_ss
        if front_ss == '11110011111' or front_ss == '11110111111' or front_ss == '11100011111':
            err = 8
        elif front_ss == '11100111111' or front_ss == '11101111111' or front_ss == '11000111111':
            err = 10
        elif front_ss == '11001111111' or front_ss == '11011111111' or front_ss == '10001111111':
            err = 12
        elif front_ss == '10011111111'  or front_ss == '10111111111' or front_ss == '00011111111':
            err = 14
        elif front_ss == '11111001111' or front_ss == '11111101111' or front_ss == '11111000111':
            err = -8
        elif front_ss == '11111100111' or front_ss == '11111110111'or front_ss == '11111100011':
            err = -10
        elif front_ss == '11111110011' or front_ss == '11111111011' or front_ss == '11111110001':
            err = -12
        elif front_ss == '11111111001' or front_ss == '11111111101' or front_ss == '11111111000':
            err = -16
        elif front_ss == '11111111110' :
            err =  - 16
        else:
            err = 0
        return err
    '''
    def error(self):
	front_ss = self.front_line_ss
        if front_ss == '00001100000' or front_ss == '00001000000' or front_ss == '00011100000':
            err = 8
        elif front_ss == '00011000000' or front_ss == '00010000000' or front_ss == '00111000000':
            err = 10
        elif front_ss == '00110000000' or front_ss == '00100000000' or front_ss == '01110000000':
            err = 12
        elif front_ss == '01100000000'  or front_ss == '01000000000' or front_ss == '11100000000':
            err = 14
        elif front_ss == '00000110000' or front_ss == '00000010000' or front_ss == '00000111000':
            err = -8
        elif front_ss == '00000011000' or front_ss == '00000001000'or front_ss == '00000011100':
            err = -10
        elif front_ss == '00000001100' or front_ss == '00000000100' or front_ss == '00000001110':
            err = -12
        elif front_ss == '00000000110' or front_ss == '00000000010' or front_ss == '00000000111':
            err = -16
        elif front_ss == '00000000001':
            err =  - 16
        else:
            err = 0
        return err

        ####################__PID_calculation__###################
    def PID_calculation(self,Kp,Kd,Ki):
        kp = Kp 
        kd = Kd
        ki = Ki
        sum_e = 0
        last_e = 0
        e_o = self.error()
        if e_o == 1:
                self.e = 0
        else :
                self.e = e_o
        sum_e = sum_e + self.e
        self.turning = Kp*self.e + Kd*(self.e - last_e) + Ki*sum_e
        #print '  ' ,'e_o = ',e_o,'e = ' , e,'turning = ',turning,'last_e = ',last_e,'back_stop = ', ss_stop,'ir_ss = ',ir_ss, '   '
        last_e = self.e
    def back_PID_calculation(self,Kp,Kd,Ki):
        sum_e = 0
        last_e = 0
        e_o = self.back_error_left()
	#print 'error = ',self.back_error_left()
        if e_o == 1:
                self.e_b = 0
        else :
                self.e_b = e_o
        sum_e = sum_e + self.e_b
        self.turning_b = Kp*self.e_b + Kd*(self.e_b - last_e) + Ki*sum_e
        #print '  ' ,'e_o = ',e_o,'e = ' , e,'turning = ',turning,'last_e = ',last_e,'back_stop = ', ss_stop,'ir_ss = ',ir_ss, '   '
        last_e = self.e_b
    def cam_PID_calculation(self,Kp,Kd,Ki):
        sum_e = 0
        last_e = 0
        e_o = self.cam_error
	#print 'error = ',self.cam_error
        if e_o == 1:
                self.e_b = 0
        else :
                self.e_b = e_o
        sum_e = sum_e + self.e_b
        self.turning_b = Kp*self.e_b + Kd*(self.e_b - last_e) + Ki*sum_e
        #print '  ' ,'e_o = ',e_o,'e = ' , e,'turning = ',turning,'last_e = ',last_e,'back_stop = ', ss_stop,'ir_ss = ',ir_ss, '   '
        last_e = self.e_b
                #####################____straight_line____####################
    def straight_line_1(self,vel):
        print 'moving straight 1'
        self.PID_calculation(15,4,0.001)
        self.angle = self.home - self.turning
        print self.angle
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        if self.angle > 4095:
            self.angle = 4095
        elif self.angle < 0:
            self.angle = 0
        self.motor_msg.x = vel
        self.motor_msg.z = self.angle
        self.vel_pub.publish(self.motor_msg)
        #####################____moving_pallet___####################
    def moving_to_pallet_back(self,speed):
        self.cam_PID_calculation(15,4,0.001)
        self.angle = self.home + self.turning_b
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        if self.angle <= 1847 and self.angle > 1747:
            self.motor_msg.x = ((speed*80)/100)
            self.motor_msg.z = self.angle
        elif self.angle <= 1747 and self.angle > 1047:
            self.motor_msg.x = ((speed*60)/100)#60%
            self.motor_msg.z = self.angle
        elif self.angle < 1047:
            self.angle = 1547
            self.motor_msg.z = self.angle
            self.motor_msg.x = ((speed*60)/100)
        elif self.angle <= 2347 and self.angle > 2247:
            self.motor_msg.x = ((speed*80)/100)
            self.motor_msg.z = self.angle
        elif self.angle >= 2347 and self.angle < 3047:
            self.motor_msg.x = ((speed*60)/100)#60%
            self.motor_msg.z = self.angle
        elif self.angle > 3047:
            self.angle = 2547
            self.motor_msg.z = self.angle
            self.motor_msg.x = ((speed*60)/100)#60%
        else:
            self.motor_msg.z = self.angle
            self.motor_msg.x = speed        
        self.vel_pub.publish(self.motor_msg)
    def moving_back(self,speed,home):
        self.cam_PID_calculation(15,4,0.001)
        self.angle = home + self.turning_b
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        if self.angle > 4095:
            self.angle = 4095
        elif self.angle < 0:
            self.angle = 0
        self.motor_msg.x = speed
        self.motor_msg.z = self.angle
        self.vel_pub.publish(self.motor_msg)
        #####################____taking_pallet___####################
    def taking_pallet(self):
        if self.test == 0:
            self.stop_cam_pub.publish('1')
            self.led_pub.publish(1)
            time.sleep(1)
            self.test = 1
        elif self.test == 1:
            if self.cam_stop == 1:
                if self.back_line_ss > 1:# '10110' or self.back_line_ss == '11100' or self.back_line_ss == '11000' or self.back_line_ss == '10000' or self.back_line_ss == '00000' or self.back_line_ss == '00011' or self.back_line_ss == '00101' or self.back_line_ss == '01001':
                    self.motor_msg.x = 0
                    self.motor_msg.z = 2047
                    self.vel_pub.publish(self.motor_msg)
                    self.test = 2
                    self.pallet = 0
                    self.cam_stop = 0
                else:
                    self.moving_to_pallet_back(-1400)
            else:
                self.moving_to_pallet_back(-1800)
        elif self.test == 2:
            self.lift_pub.publish("lift_down")
            if self.lift_val == 2:
                self.lift_pub.publish("lift_stop")
                self.pos_encoder = self.encoder
                self.test = 3                
        elif self.test == 3:
            self.encoder_now = self.encoder
            self.encoder_value = int(self.encoder_now - self.pos_encoder)
            print self.encoder_value
            if  self.encoder_value <= self.encoder_home :
            #self.count = self.count + 1
            #if self.count == 150:
                #self.count = 0
                self.pos_encoder = 0
                self.encoder_value = 0
                self.test = 4
            else:
                self.moving_to_pallet_back(-1000)                
        elif self.test == 4:
            self.motor_msg.x = 0
            self.motor_msg.z = 2047
            self.vel_pub.publish(self.motor_msg)
            self.test = 5
        elif self.test == 5:
            self.lift_pub.publish("lift_up")
            if self.lift_val == 1:
                self.lift_pub.publish("lift_stop")
                self.led_pub.publish(0)
                self.stop_cam_pub.publish('stop')
                self.test = 9
        elif self.test == 6:
            if self.line > 1:
                self.test = 7
            else:
                self.motor_msg.x = 800
                self.motor_msg.z = 1547
                self.vel_pub.publish(self.motor_msg)
        elif self.test == 7:
            self.count = self.count + 1
            if self.count == 40:
                self.count = 0
                self.test = 8
            else:
                self.straight_line_1(800)
        elif self.test == 8:
            if self.front_line_ss == '00000000000' or self.front_line_ss == '11111111111':
                self.count = self.count + 1
                if self.count == 30:
                    self.test = 9
                else:
                    self.straight_line_1(2000)
            else:
                self.count = 0
                self.straight_line_1(2000)
        elif self.test == 9:
            self.status = 1
            self.motor_msg.x = 0
            self.motor_msg.z = 2047
            self.vel_pub.publish(self.motor_msg)
            self.test = 0
            self.PID_enable = 2         
            
        #####################____put_pallet_down___####################
    def put_down_pallet(self):
        if self.test == 0:
            self.stop_cam_pub.publish('1')
            self.led_pub.publish(1)
            time.sleep(1)
            self.test = 1
        elif self.test == 1:
            if self.cam_stop == 1:
		print self.back_line_ss
                if self.back_line_ss > 1:# '10110' or self.back_line_ss == '11100' or self.back_line_ss == '11000' or self.back_line_ss == '10000' or self.back_line_ss == '00000' or self.back_line_ss == '00011' or self.back_line_ss == '00101' or self.back_line_ss == '01001':
                    self.motor_msg.x = 0
                    self.motor_msg.z = 2047
                    self.vel_pub.publish(self.motor_msg)
                    self.test = 2
                    self.pallet = 0
                    self.cam_stop = 0
                else:
                    self.moving_to_pallet_back(-1400)                
            else:
                self.moving_to_pallet_back(-1800)
        elif self.test == 2:
            self.lift_pub.publish("lift_down")
            if self.lift_val == 2:
                self.lift_pub.publish("lift_stop")
                self.led_pub.publish(0)
                self.stop_cam_pub.publish('stop')
                self.pos_encoder = self.encoder
                self.test = 3
        elif self.test == 3:
            self.encoder_now = self.encoder
            self.encoder_value = int(self.pos_encoder - self.encoder_now )
            print self.encoder_value
            if  self.encoder_value <= self.encoder_home :
            #self.count = self.count + 1
            #if self.count == 150:
                #self.count = 0
                self.pos_encoder = 0
                self.encoder_value = 0
                self.test = 4
            else:
                print self.r_line,type(self.r_line)
                #if self.r_line == '01111' or self.r_line == '00111' or self.r_line == '10111' or self.r_line == '10011' or self.r_line == '11011' or self.r_line == '11001' or self.r_line == '11101'or self.r_line == '11100':
                if self.r_line == '10000' or self.r_line == '11000' or self.r_line == '01000' or self.r_line == '01100' or self.r_line == '00100' or self.r_line == '00110' or self.r_line == '00010'or self.r_line == '00011':
                    self.motor_msg.x = 1000
                    self.motor_msg.z = 2297
                    self.vel_pub.publish(self.motor_msg)
                else:
                    self.motor_msg.x = 1000
                    self.motor_msg.z = 2047
                    self.vel_pub.publish(self.motor_msg)
        elif self.test == 4:
            self.motor_msg.x = 0
            self.motor_msg.z = 2047
            self.vel_pub.publish(self.motor_msg)
            self.test = 5
        elif self.test == 5:
            self.lift_pub.publish("lift_up")
            if self.lift_val == 1:
                self.lift_pub.publish("lift_stop")
                self.test = 9
        elif self.test == 6:
            if self.line > 1:
                self.test = 7
            else:
                self.motor_msg.x = 800
                self.motor_msg.z = 1547
                self.vel_pub.publish(self.motor_msg)
        elif self.test == 7:
            self.count = self.count + 1
            if self.count == 40:
                self.count = 0
                self.test = 8
            else:
                self.straight_line_1(800)
        elif self.test == 8:
            if self.front_line_ss == '00000000000' or self.front_line_ss == '11111111111':
                self.count = self.count + 1
                if self.count == 30:
                    self.test = 9
                else:
                    self.straight_line_1(2000)
            else:
                self.count = 0
                self.straight_line_1(2000)
        elif self.test == 9:
            self.status = 2
            self.motor_msg.x = 0
            self.motor_msg.z = 2047
            self.vel_pub.publish(self.motor_msg)
            self.test = 0
            self.PID_enable = 2    
        ##############################____MAIN____#########################
    def main_run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.PID_enable == 1:
                #self.curve_right_line()
                #self.moving_to_pallet_back(-800)
                #self.taking_pallet(self.pallet_num)
                #self.straight_line_1(800)
                #'''
                if self.stop_flag >= 10 and self.flag == 0:
                    self.motor_msg.x = 0
                    self.vel_pub.publish(self.motor_msg)
                    print 'have object in front from 0 to 180 degree'
                    self.flag = 1
                elif self.flag == 1:
                    if self.stop_flag < 10:
                        self.flag = 0
                    else:
                        self.motor_msg.x = 0
                        self.vel_pub.publish(self.motor_msg)
                        print 'have object in front from 0 to 180 degree'
                else:
                    if self.stop == 'stop':
                        self.motor_msg.x = 0
                        self.motor_msg.z = 2047
                        self.vel_pub.publish(self.motor_msg)
                        self.PID_enable = 2
                    else:
                        if self.temp == 0:
                            if self.cx == 1:
                                self.temp = 2
                            elif self.cx == 2:
                                self.temp = 1
                                print 'no line'
                                self.noline_pub.publish('no line')
                        elif self.temp == 1:
                            if self.turn == 'turn left':
                                if self.cx == 1:
                                    self.temp = 2
                                    self.motor_msg.x = 0
                                    self.motor_msg.z = 2347
                                    self.vel_pub.publish(self.motor_msg)
                                else:
                                    self.motor_msg.x = -800
                                    self.motor_msg.z = 2347
                                    self.vel_pub.publish(self.motor_msg)
                            elif self.turn == 'turn right':
                                if self.cx == 1:
                                    self.temp = 2
                                    self.motor_msg.x = 0
                                    self.motor_msg.z = 1747
                                    self.vel_pub.publish(self.motor_msg)
                                else:
                                    self.motor_msg.x = -800
                                    self.motor_msg.z = 1747
                                    self.vel_pub.publish(self.motor_msg)
                        elif self.temp == 2:
                            print 'taking pallet'
                            self.taking_pallet()
                #'''
            elif self.PID_enable == 2:
                if self.status == 1:
                    self.program_pub.publish(3203)
                elif self.status == 2:
                    self.program_pub.publish(3204)
                self.led_pub.publish(0)
                self.laser_data = []
                self.PID_enable = 0
                self.rfid = 0
                self.lift_val = 3
                self.stop_flag = 0
                self.flag = 0
                self.flag_1 = 0
                self.flag_2 = 0
                self.status = 0
                self.right_ss = 0
                self.left_ss = 0
                self.ir_ss = 0
                self.back_line_ss = 0
                self.turning = 0
                self.turning_b = 0
                self.e = 0
                self.e_b = 0
                self.e_b = 0
                self.home = 2047
                self.angle = 0
                self.vel = 0
                self.right_value = 1397
                self.tt = 0
                self.vel_value = 2947
                self.left_value1 = 2497
                self.test = 0
                self.temp = 0
                self.motor_msg = Vector3()
                self.count = 0
                self.count_1 = 0
                self.rig_home = 1397
                self.motor_msg.x = 0
                self.motor_msg.z = 2047
                self.pallet = 0
                self.vel_pub.publish(self.motor_msg)
                self.cam_home = 314#151# 88
                self.cam_error = 0
                self.rec = 0
                self.control = 0
                self.cam_stop = 0
                self.pallet_num = 0
                self.right_stop = 0
                self.server = 0
                self.encoder = 0
                self.encoder_home = -3600
                self.encoder_now = 0
                self.pos_encoder = 0
                self.encoder_value = 0
                self.position = 0
                self.stop = 0
                self.line = 0
                self.r_line = 0
                self.cx = 0
                self.tem = 1
                self.count_temp = 0
                self.turn = 0
                self.stop_cam_pub.publish('1')
            elif self.PID_enable == 3:
                if self.stop_flag >= 10 and self.flag == 0:
                    self.motor_msg.x = 0
                    self.vel_pub.publish(self.motor_msg)
                    print 'have object in front from 0 to 180 degree'
                    self.flag = 1
                elif self.flag == 1:
                    if self.stop_flag < 10:
                        self.flag = 0
                    else:
                        self.motor_msg.x = 0
                        self.vel_pub.publish(self.motor_msg)
                        #print 'have object in front from 0 to 180 degree'
                else:
                    if self.stop == 'stop':
                        self.motor_msg.x = 0
                        self.motor_msg.z = 2047
                        self.vel_pub.publish(self.motor_msg)
                        self.PID_enable = 2
                    else:
                        if self.temp == 0:
                            if self.cx == 1:
                                self.temp = 2
                            elif self.cx == 2:
                                self.temp = 1
                                print 'no line'
                                self.noline_pub.publish('no line')
                        elif self.temp == 1:
                            if self.turn == 'turn left':
                                if self.cx == 1:
                                    self.temp = 2
                                    self.motor_msg.x = 0
                                    self.motor_msg.z = 2347
                                    self.vel_pub.publish(self.motor_msg)
                                else:
                                    self.motor_msg.x = -800
                                    self.motor_msg.z = 2347
                                    self.vel_pub.publish(self.motor_msg)
                            elif self.turn == 'turn right':
                                if self.cx == 1:
                                    self.temp = 2
                                    self.motor_msg.x = 0
                                    self.motor_msg.z = 1747
                                    self.vel_pub.publish(self.motor_msg)
                                else:
                                    self.motor_msg.x = -800
                                    self.motor_msg.z = 1747
                                    self.vel_pub.publish(self.motor_msg)
                        elif self.temp == 2:
                            print 'put down pallet'
                            self.put_down_pallet()
            elif self.PID_enable == 0:
                print ' Waiting for taking or put down pallet '
                #print 'self.pallet_num = ',self.pallet_num,type(self.pallet_num)
                #pass
            r.sleep()
            
if __name__ == '__main__':
    run = Echo()
    run.main_run()
