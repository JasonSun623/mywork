#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String 

ser = serial.Serial(

	port='/dev/ttyUSB0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
    )

def talker():
    pub = rospy.Publisher('button_call', String, queue_size=10)
    rospy.init_node('ButtonCall', anonymous=True)
    while not rospy.is_shutdown():
        x=ser.readline().decode()
        print(x)
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass