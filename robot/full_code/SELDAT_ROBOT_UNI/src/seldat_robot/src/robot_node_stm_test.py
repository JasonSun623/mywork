#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    PC takes messages on MCU
    Author: Robot Team
      
"""

import rospy
import serial
import sys
#import os
import time
import math
import numpy
#import std_msgs.msg
from std_msgs.msg import Int32,UInt32,String,Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3, Quaternion
from rospy.numpy_msg import numpy_msg
from subprocess import call
#import dbus

serial = serial.Serial("/dev/ttyACM0", baudrate = 115200)

class STM32_class(object):

    def __init__(self):
        print "Initializing STM32 Class..."
        '''
        rospy.init_node("STM32_COM")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = rospy.get_param('~rate', 200)
        '''
        self.encoderMsg		= Vector3()

        self.magneticfrontMsg	= String()
        self.magneticbackMsg	= String()
        self.magneticleftMsg	= String()
        self.magneticrightMsg	= String()

        self.manualMsg		= Int32()		
        self.liftMsg		= UInt32()
        self.battvoltageMsg	= Float32()
        self.sonarMsg		= Range()
        self.errorMsg		= Int32()
        self.warningMsg		= Int32()		
        self.changeCTMsg	= Int32()
        #self.stopMsg		= Int32()
        self.poweroffMsg	= Int32()
        self.magOnMsg		= False

        self.encoders_pub 	= rospy.Publisher('pos',Vector3,queue_size = 10)

        self.magnetic_front_pub	= rospy.Publisher('magnetfront',String,queue_size = 10)
        self.magnetic_back_pub 	= rospy.Publisher('magnetback',String,queue_size = 10)
        self.magnetic_left_pub	= rospy.Publisher('magnetleft',String,queue_size = 10)
        self.magnetic_right_pub = rospy.Publisher('magnetright',String,queue_size = 10)

        self.manual_pub		= rospy.Publisher('manual',Int32,queue_size = 10)
        self.lift_pub		= rospy.Publisher('lift',UInt32,queue_size = 10)
        self.batt_pub		= rospy.Publisher('battery_vol',Float32,queue_size = 10)
        self.error_pub		= rospy.Publisher('stm_error',Int32,queue_size = 10)
        self.warning_pub	= rospy.Publisher('stm_warning',Int32,queue_size = 10)
        self.changeCT_pub	= rospy.Publisher('chargeCT',Int32,queue_size = 10)

        #self.encoderMsg.header  = std_msgs.msg.Header()
        #self.sonarMsg.header.stamp = rospy.Time.now()

        #self.frameid = ['/ultrasound']	
        self.sonarMsg.header.frame_id 	=  '/ultrasound'
        self.sonarMsg.field_of_view 	= 0.5  # approximately
        self.sonarMsg.radiation_type 	= Range.ULTRASOUND
        self.sonarMsg.min_range 	= 0.15
        self.sonarMsg.max_range 	= 6.4

        self.ir_right_pub 	= rospy.Publisher('ir_right',Range,queue_size = 10)
        self.ir_left_pub 	= rospy.Publisher('ir_left',Range,queue_size = 10)
        self.sonar_right_pub 	= rospy.Publisher('sonar_right',Range,queue_size = 10)
        self.sonar_front_pub 	= rospy.Publisher('sonar_front',Range,queue_size = 10)
        self.sonar_left_pub 	= rospy.Publisher('sonar_left',Range,queue_size = 10)
		
        rospy.Subscriber("twheel", Int32, self.twheelCallback)
        rospy.Subscriber("swheel", Int32, self.rwheelCallback)
        rospy.Subscriber("swheel_rtarget",Float32, self.rwheelRadCallback)
        rospy.Subscriber("lift_control", String, self.liftCallback)
        rospy.Subscriber("ctrlRobotHardware", Int32, self.chargeCallback)
        rospy.Subscriber("wifi_check", Int32, self.wifiCallback)
        rospy.Subscriber("linedetectionctrl", Int32, self.LineDetectionCtrlCallback)
        rospy.Subscriber("linedetectioncallback", Int32, self.LineDetectionCallback)
        rospy.Subscriber("errorDetectedLine", Int32, self.ErrorDetectedLineCallback)
        rospy.Subscriber("stop_flag", Int32, self.obstaclesCallback)
	
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.Update_sensors()
            r.sleep()
	
    def Update_sensors(self):
        #data = serial.readline().rstrip()
        data = serial.read()
        if (data == 's') or (data == 'a') or (data == 'm'):
            check = 0
            while (1):
                data_n = serial.read()
                if data_n == 'e':
                    break
                elif data_n != 'e':
                    data = data + data_n

            if(len(data) > 0):
                #print(data)
                lineParts = data.split('\t')
                try:
                    if(lineParts[0] == 's'):
                        self.encoderMsg.x = long(lineParts[1]) #tracking
                        self.encoderMsg.z = long(lineParts[2]) #rotating
                        self.encoders_pub.publish(self.encoderMsg)
                    elif(lineParts[0] == 'a'):
                        for i in range(1,6):
                            self.sonarMsg.range = float(lineParts[i])/1000
                            if (i == 1):
                                self.sonar_right_pub.publish(self.sonarMsg)
                            elif (i == 2):
                                self.sonar_front_pub.publish(self.sonarMsg)
                            elif (i == 3):
                                self.sonar_left_pub.publish(self.sonarMsg)
                            elif (i == 4):
                                self.ir_right_pub.publish(self.sonarMsg)
                            elif (i == 5):
                                self.ir_left_pub.publish(self.sonarMsg)
                        self.battvoltageMsg = float(lineParts[1])
                        self.batt_pub.publish(self.battvoltageMsg)
			
			self.liftMsg = int(lineParts[4])
                        self.lift_pub.publish(self.liftMsg)
                       
                        self.changeCTMsg = int(lineParts[9])
                        self.changeCT_pub.publish(self.changeCTMsg)

                    elif(lineParts[0] == 'm'):
			#print(data)
                        self.magneticfrontMsg 	= str(lineParts[1])
                        self.magneticbackMsg 	= str(lineParts[2])
                        self.magneticleftMsg 	= str(lineParts[3])
                        self.magneticrightMsg 	= str(lineParts[4])
			
                        self.magnetic_front_pub.publish(self.magneticfrontMsg)
                        self.magnetic_back_pub.publish(self.magneticbackMsg)
                        self.magnetic_left_pub.publish(self.magneticleftMsg)
                        self.magnetic_right_pub.publish(self.magneticrightMsg)

			#self.liftMsg = int(lineParts[2])
                        #self.lift_pub.publish(self.liftMsg)

                        #self.manualMsg = int(lineParts[3])
                        #self.manual_pub.publish(self.manualMsg)

                        #self.errorMsg = int(lineParts[4])
                        #self.error_pub.publish(self.errorMsg)

                        #self.warningMsg = int(lineParts[5])
                        #self.warning_pub.publish(self.warningMsg)

                        #self.magnetlinesaddMsg = str(lineParts[6])

                        #self.changeCTMsg = int(lineParts[7])
                        #self.changeCT_pub.publish(self.changeCTMsg)
			'''
                        self.poweroffMsg = int(lineParts[8])
                        if (self.poweroffMsg == 1):
                            rospy.loginfo("====Computer Shutdown====")
                            time.sleep(1)
                            call("sudo shutdown -h now", shell=True)
                        elif (self.poweroffMsg == 2):
                            rospy.loginfo("====Computer Restart=====")
                            time.sleep(1)
                        #    os.system("shutdown now -h")
                            call("sudo reboot -h now", shell=True)
                        if (self.magOnMsg == True):
                            self.magnet_lines_pub.publish(self.magnetlinesMsg)
                            self.magnet_lines_add_pub.publish(self.magnetlinesaddMsg)
			'''
                except:
                    print (data)
                    print ("============Loi ne ba==========")
                    pass
				
    def twheelCallback(self, msg):
        t_vel =  "T" + str(msg.data) +"e"
        serial.write(t_vel)
        #print (t_vel)
    def rwheelCallback(self, msg):
        r_vel =  "R" + str(msg.data) +"e"
        serial.write(r_vel)
        #print (r_vel)
    def rwheelRadCallback(self,msg):
        r_vel_rad = "R" + str(int(5300 - (3183*msg.data))) + "e" #3482 3374
        serial.write(r_vel_rad)
        #print (r_vel_rad)
    def liftCallback(self,msg):
        data = msg.data
        if data == "lift_up":
            serial.write("L1e")
        elif data == "lift_down":
            serial.write("L2e")
        elif data == "lift_stop":
            serial.write("L0e")
    def chargeCallback(self,msg):
        charge_cmd = "C" + str(msg.data) + "e"
        serial.write(charge_cmd)
    def wifiCallback(self,msg):
        wifi_msg = "E" + str(msg.data) + "e"
        serial.write(wifi_msg)

    def LineDetectionCtrlCallback(self,msg):
        if (msg.data == 1203) or (msg.data == 1204) or (msg.data == 1206) or (msg.data == 1208):
            self.magOnMsg = True

    def LineDetectionCallback(self,msg):
        if (msg.data == 3203) or (msg.data == 3204):
            self.magOnMsg = False

    def ErrorDetectedLineCallback(self,msg):
        if (msg.data == 4205):
            self.magOnMsg = False
	    
    def obstaclesCallback(self,msg):
        if (msg.data == 1002):
            obst =  "W2001e"
            serial.write(obst)
        else :
            obst =  "W2000e"
            serial.write(obst)
            #print(obst)

if __name__ == '__main__':
	
    """ main """
    '''
    stm32 = STM32_class()
    stm32.spin()
	
    '''
    rospy.init_node('STM32_COM', anonymous=True)
    stm32 = STM32_class()
    try:
        while not rospy.is_shutdown():
            stm32.Update_sensors()
    except rospy.ROSInterruptException:
        rospy.logwarn("Error in main function")
