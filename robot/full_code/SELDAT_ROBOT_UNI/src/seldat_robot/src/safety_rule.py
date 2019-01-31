#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    PC takes messages on Lidar
    Author: Robot Team
      
"""
import rospy
import sys
import time
import numpy
from math import sin, cos, pi, atan
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Vector3, Twist # Quaternion
from sensor_msgs.msg import LaserScan

class safety_rule(object):
    
    def __init__(self):
        #print "Initializing Safety Class..."
		
        rospy.init_node("SAFETY_NODE")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = rospy.get_param('~rate', 10)

        self.stop_dist_fast	= 1.7	#(m)
        self.warn_dist_fast	= 5
        self.stop_dist_slow	= 1.2
        self.warn_dist_slow	= 3
        self.side_dist		= 1.0
        self.warn_speed		= 0.5 #(m/s)
        self.twheel_cmd		= 0.0
        self.rwheel_radian	= 0.0
        self.danger_fast	= False
        self.warning_fast	= False
        self.danger_slow	= False
        self.warning_slow	= False
        self.magOnMsg		= False

        self.stopflagMsg	= Int32()
        self.radianMsg		= Float32()
        self.laserToPosMsg	= numpy.zeros((181,2))
        self.scanMsg		= LaserScan()
        self.stop_pub		= rospy.Publisher('stop_flag',Int32,queue_size = 10)
		
        rospy.Subscriber('/scan', LaserScan, self.ScanCallBack)
        rospy.Subscriber("cmd_vel_mux/input/navi", Twist, self.CmdCallback)
        rospy.Subscriber("linedetectionctrl", Int32, self.LineDetectionCtrlCallback)
        rospy.Subscriber("linedetectioncallback", Int32, self.LineDetectionCallback)
        rospy.Subscriber("errorDetectedLine", Int32, self.ErrorDetectedLineCallback)
    
    
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.Update_Sensors()
            r.sleep()
    
    def Update_Sensors(self):
        if (self.magOnMsg == False):
            #print(self.twheel_cmd)
            if ((self.twheel_cmd <= self.warn_speed)and(self.twheel_cmd > 0)):
                for i in range(0, 181):
                    if ((self.laserToPosMsg[i][0] <= self.stop_dist_slow)and(self.laserToPosMsg[i][1] <= self.side_dist)and(self.laserToPosMsg[i][1] >= -self.side_dist)):
                        self.danger_slow = True;
                    elif ((self.laserToPosMsg[i][0] <= self.warn_dist_slow)and(self.laserToPosMsg[i][1] <= self.side_dist)and(self.laserToPosMsg[i][1] >= -self.side_dist)):
                        self.warning_slow = True;
            elif (self.twheel_cmd > self.warn_speed):
                for i in range(0, 181):
                    if ((self.laserToPosMsg[i][0] <= self.stop_dist_fast)and(self.laserToPosMsg[i][1] <= self.side_dist)and(self.laserToPosMsg[i][1] >= -self.side_dist)):
                        self.danger_fast = True;
                    elif ((self.laserToPosMsg[i][0] <= self.warn_dist_fast)and(self.laserToPosMsg[i][1] <= self.side_dist)and(self.laserToPosMsg[i][1] >= -self.side_dist)):
                        self.warning_fast = True;
        
            if (self.danger_slow == True):
                self.stopflagMsg = 1002
                self.stop_pub.publish(self.stopflagMsg)
                self.danger_slow = False
                rospy.loginfo("===Stop=== detected obstacles at low speed")
                #print("danger slow")
			
            elif (self.warning_slow == True):
                self.stopflagMsg = 1402
                self.stop_pub.publish(self.stopflagMsg)
                self.warning_slow  = False
                rospy.loginfo("===Slow=== Detected obstacles at low speed")
                #print("warning slow")
            elif (self.danger_fast == True):
                self.stopflagMsg = 1002
                self.stop_pub.publish(self.stopflagMsg)
                self.danger_fast = False
                rospy.loginfo("===Stop=== Detected obstacles at high speed")
                #print("danger fast")
            elif (self.warning_fast == True):
                self.stopflagMsg = 1402
                self.stop_pub.publish(self.stopflagMsg)
                self.warning_fast = False
                rospy.loginfo("===Slow=== Detected obstacles at high speed")
                #print("warning fast")
            else:
                self.stopflagMsg = 1802
                self.stop_pub.publish(self.stopflagMsg)
        else:
            self.stopflagMsg = 1802
            self.stop_pub.publish(self.stopflagMsg) 
		
    def ScanCallBack(self, msg):
        self.scanMsg = msg.ranges
        for i in range(0, len(self.scanMsg)):
            self.laserToPosMsg[i][0] = self.scanMsg[i] * sin((pi*i)/180);
            self.laserToPosMsg[i][1] = self.scanMsg[i] * -cos((pi*i)/180);
        #print (len(self.scanMsg))

    def CmdCallback(self, msg):
        if (msg.linear.x != 0):
            self.rwheel_radian = atan((1.136*msg.angular.z)/msg.linear.x)
            self.twheel_cmd = (msg.linear.x/cos(self.rwheel_radian))
        else:
            self.twheel_cmd = 0
		
    def LineDetectionCtrlCallback(self,msg):
        if (msg.data == 1203) or (msg.data == 1204) or (msg.data == 1206) or (msg.data == 1208):
            self.magOnMsg = True

    def LineDetectionCallback(self,msg):
        if (msg.data == 3203) or (msg.data == 3204):
            self.magOnMsg = False

    def ErrorDetectedLineCallback(self,msg):
        if (msg.data == 4205):
            self.magOnMsg = False

if __name__=='__main__':
    """ main """

    safety = safety_rule()
    safety.spin()
    '''
    rospy.init_node('Safety_Node', anonymous=True)
    safety = safety_rule()
    try:
        while not rospy.is_shutdown():
            safety.Update_Sensors()
    except rospy.ROSInterruptException:
            rospy.logwarn("Error in main function")
    '''			
#print( 'connected' if connected() else 'no internet!' )
