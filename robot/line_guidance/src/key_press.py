#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import curses
from std_msgs.msg import UInt16,UInt8,Int8,Int32, UInt32, String,UInt8MultiArray,Float32


pub = rospy.Publisher('key_press', String, queue_size=10)
#t_pub = rospy.Publisher('twheel', Int32,queue_size = 10)
cmd_pub = rospy.Publisher('linedetectionctrl', Int32,queue_size = 10)
rospy.init_node('key_node')
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.refresh()

key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key ==ord('p'):
        cmd_pub.publish(1203)
        pub.publish('key p is press')
	#t_pub.publish(-600)
        #print 'key p is press'
    elif key == ord('m') : 
        cmd_pub.publish(1204)
        #print 'm is press'
#	pub.publish('key m is press')
    elif key == ord('s') :
        #print 'm is press'
        pub.publish('stop pid')
	#t_pub.publish(0)
#    elif key == ord('l') :
#        pub.publish('key l is press')
#    elif key == ord('k'):
#	pub.publish('key k is press')
#    elif key == ord('j'):
#	pub.publish('key j is press')

curses.endwin()
