#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
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

serial = serial.Serial("/dev/STM32F4", baudrate = 115200)

class STM32_class(object):

	def __init__(self):
		print "Initializing STM32 Class..."
		'''
		rospy.init_node("STM32_COM")
		self.nodename = rospy.get_name()
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate', 100)
		'''
		self.encoderMsg		= Vector3()
		self.magnetlinesMsg	= String()
		self.magnetlinesaddMsg	= String()
		self.manualMsg		= Int32()		
		self.liftMsg		= UInt32()
		self.battvoltageMsg	= Float32()
		self.sonarMsg		= Range()
		self.obstacleMsg	= Int32()
		self.changeCTMsg	= Int32()
		self.stopMsg		= Int32()
		self.poweroffMsg	= Int32()

		self.encoders_pub 	= rospy.Publisher('pos',Vector3,queue_size = 10)
		self.magnet_lines_pub	= rospy.Publisher('magnetline',String,queue_size = 10)
		self.magnet_lines_add_pub = rospy.Publisher('magnetlineadd',String,queue_size = 10)
		self.manual_pub		= rospy.Publisher('manual',Int32,queue_size = 10)
		self.lift_pub		= rospy.Publisher('lift',UInt32,queue_size = 10)
		self.batt_pub		= rospy.Publisher('battery_vol',Float32,queue_size = 10)
		self.obstacle_pub	= rospy.Publisher('stop_flag_2',Int32,queue_size = 10)
		self.changeCT_pub	= rospy.Publisher('chargeCT',Int32,queue_size = 10)
		self.stop_pub		= rospy.Publisher('robot_stop',Int32,queue_size = 10)

		#self.encoderMsg.header  = std_msgs.msg.Header()
		#self.sonarMsg.header.stamp = rospy.Time.now()

		#self.frameid = ['/ultrasound']	
  		self.sonarMsg.header.frame_id =  '/ultrasound'
  		self.sonarMsg.field_of_view = 0.5  # approximately
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
		rospy.Subscriber("lift_controll", String, self.liftCallback)
		rospy.Subscriber("ctrlRobotHardware", Int32, self.chargeCallback)
                rospy.Subscriber("wifi_check", Int32, self.wifiCallback)
	
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
                        self.Update_sensors()
                        r.sleep()
	

	def Update_sensors(self):
		#data = serial.readline().rstrip()
		data = serial.read()
		if (data == 's') or (data == 'a') or (data == 'l'):
			check = 0
			while (1):
				data_n = serial.read()
				if data_n == 'e':
					break
				elif data_n != 'e':
					data = data + data_n

			if(len(data) > 0):
				lineParts = data.split('\t')
				try:
 					if(lineParts[0] == 's'):
						self.encoderMsg.x = long(lineParts[1]) #tracking
						self.encoderMsg.z = long(lineParts[2]) #rotating
						self.encoders_pub.publish(self.encoderMsg)
						self.stopMsg	= int(lineParts[3])
						self.stop_pub.publish(self.stopMsg)
						#print(self.encoderMsg.x)

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

						self.battvoltageMsg = float(lineParts[6])
                                                self.batt_pub.publish(self.battvoltageMsg)

					elif(lineParts[0] == 'l'):
                                                self.magnetlinesMsg = str(lineParts[1])
                                                self.magnet_lines_pub.publish(self.magnetlinesMsg)

                                                self.liftMsg = int(lineParts[2])
                                                self.lift_pub.publish(self.liftMsg)

                                                self.manualMsg = int(lineParts[3])
                                                self.manual_pub.publish(self.manualMsg)

						self.obstacleMsg = int(lineParts[4])
						self.obstacle_pub.publish(self.obstacleMsg)

						self.magnetlinesaddMsg = str(lineParts[5])
						self.magnet_lines_add_pub.publish(self.magnetlinesaddMsg)

						self.changeCTMsg = int(lineParts[6])
						self.changeCT_pub.publish(self.changeCTMsg)

						self.poweroffMsg = int(lineParts[7])
						if (self.poweroffMsg == 1002):
							print("====Shutdown====")
							time.sleep(5)
							#call("sudo shutdown -h now", shell=True)
							#sys_bus = dbus.SystemBus()
							#ck_srv = sys_bus.get_object('org.freedesktop.ConsoleKit',
                            				#			'/org/freedesktop/ConsoleKit/Manager')
							#ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.ConsoleKit.Manager')
							#stop_method = ck_iface.get_dbus_method("Stop")
							#stop_method()
						
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
		r_vel_rad = "R" + str(int(5400 - (3482*msg.data))) + "e"
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
                wifi_msg = "I" + str(msg.data) + "e"
                serial.write(wifi_msg)


if __name__ == '__main__':
	'''
	""" main """
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
