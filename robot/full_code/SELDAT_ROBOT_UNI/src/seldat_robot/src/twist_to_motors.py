#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    twist_to_motors - converts a twist message to motor commands.
    Author: Robot Team
      
"""

import rospy
import roslib
from math import cos, atan
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.37)
    
        self.pub_fmotor = rospy.Publisher('fwheel_vtarget', Float32, queue_size=10)
        self.pub_smotor = rospy.Publisher('swheel_rtarget', Float32, queue_size=10)
        rospy.Subscriber('cmd_vel_mux/input/navi', Twist, self.twistCallback)
        rospy.Subscriber("stop_flag", Int32, self.obstaclesCallback)
        rospy.Subscriber("line_stop_rad", Float32, self.stopRadCallback)
        rospy.Subscriber("manual", Int32, self.manualCallback)
        rospy.Subscriber("stm_error", Int32, self.stmErrorCallback)
        rospy.Subscriber("stm_warning", Int32, self.stmWarnCallback)
        rospy.Subscriber("ctrlRobotDriving", Int32, self.serverCallback)
        rospy.Subscriber("wifi_check", Int32, self.wifiCallback)
		
        self.rate = rospy.get_param("~rate", 30) #50
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.d = 0.8
        self.stop_flag = 1802
        self.stop_rad = 0
        self.manual_flag = 0
        self.stm_error = 1000
        self.stm_warn = 2000
        self.server_speed = 100
        self.dr = Float32()
        self.dx = Float32()
        self.angle = Float32()
        self.front = Float32()
        self.wifi_msg = Int32()
		
        self.flag_enable = 1002
        self.flag_disable = 1802
        self.flag_1402  = 1402
        self.slowdown   = 0.5
		
        self.obstacles = 2001
        self.obst_lowbatt = 2012

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
        if (self.dx != 0):
            self.angle = atan((self.d * self.dr)/self.dx);
            self.front = (self.dx/cos(self.angle))*(self.server_speed/100)
        elif ((self.dx == 0)and(self.stop_rad == 0)):
            self.angle = 0
            self.front = 0
        elif ((self.dx == 0)and(self.stop_rad != 0)):
            self.angle = self.stop_rad
            self.front = 0
        
        if (self.manual_flag == 1):
            self.front = 0
        elif (self.manual_flag == 0):
            if ((self.stop_flag == self.flag_1402)and(self.server_speed > self.slowdown)):
                if (self.front > self.slowdown):
                    self.front = self.slowdown
            elif (self.stop_flag == self.flag_enable):
                self.front = 0
				
            if ((self.stm_warn == self.obstacles)or(self.stm_warn == self.obst_lowbatt)):
                self.front = 0
            #if (self.wifi_msg != 1000):
            #    self.front = 0
            if (self.stm_error != 1000):
                self.front = 0
		rospy.loginfo("-D- Traction: %0.2f Rotation: %0.2f" % (self.front,self.angle))
        self.pub_fmotor.publish(self.front)
        self.pub_smotor.publish(self.angle)
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
    def obstaclesCallback(self,msg):
        self.stop_flag = msg.data
    def stopRadCallback(self,msg):
        self.stop_rad = msg.data

    def manualCallback(self,msg):
        self.manual_flag = msg.data
    def stmErrorCallback(self,msg):
        self.stm_error = msg.data
    def stmWarnCallback(self,msg):
        self.stm_warn = msg.data
    def serverCallback(self,msg):
        self.server_speed = msg.data	
    def wifiCallback(self,msg):
        self.wifi_msg = msg.data
		
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
