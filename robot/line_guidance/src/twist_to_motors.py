#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    twist_to_motors - converts a twist message to motor commands.
    Author: Robot Team
      
"""

import rospy
import roslib
from math import cos, atan
from std_msgs.msg import Float32, Int32, String
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
        #rospy.Subscriber("stop_flag", Int32, self.obstaclesCallback)
        rospy.Subscriber("line_stop_rad", Float32, self.stopRadCallback)
        rospy.Subscriber("manual", Int32, self.manualCallback)
        rospy.Subscriber("stm_error", String, self.stmErrorCallback)
        rospy.Subscriber("stm_warning", String, self.stmWarnCallback)
        rospy.Subscriber("ctrlRobotDriving", Int32, self.serverCallback)
        rospy.Subscriber("wifi_check", String, self.wifiCallback)
        rospy.Subscriber("run_button", Int32, self.runbuttonCallback)
        rospy.Subscriber("goalChecker", Int32, self.goalcheckCallback)
		
        self.rate = rospy.get_param("~rate", 30) #50
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.d = 0.79
        #self.stop_flag = 1802
        self.stop_rad = 0
        self.manual_flag = 0
        self.stm_error = 0
        self.stm_warn = 0
        self.server_speed = 100
        self.dr = Float32()
        self.dx = Float32()
        self.angle = Float32()
        self.front = Float32()
        self.wifi_msg = Int32()
        self.run_button = Int32()
        self.goal_check = Int32()
		
        #self.flag_enable = 10020
        #self.flag_disable = 1802
        #self.flag_1402  = 1402
        self.slowdown    = 0.5
        self.distance_to_goal = 7
		
        self.slow_flag = False
        self.slow_accelerate = 0.05
        self.slow_memset = 0
        #self.obstacles = 2001
        #self.obst_lowbatt = 2012

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
            if (self.angle >= 0.3)or(self.angle <= -0.3):
                self.front = self.front*0.7
        elif ((self.dx == 0)and(self.stop_rad == 0)):
            self.angle = 0
            self.front = 0
        elif ((self.dx == 0)and(self.stop_rad != 0)):
            self.angle = self.stop_rad
            self.front = 0
			
        #########################################################
        if (self.manual_flag == 1):
            self.front = 0
        elif (self.manual_flag == 0):
            #if (self.goal_check <= 7):
            #    if (self.front > self.slowdown_goal):
            #        self.front = self.slowdown_goal
			
            # ===========warning slow============
            if ((self.stm_warn[0] == "1")or(self.goal_check <= self.distance_to_goal))and(self.front > self.slowdown):
                #self.front = self.slowdown
                if (self.slow_flag == False):
                    self.slow_memset = self.front
                    self.front = self.slow_memset - self.slow_accelerate
                    self.slow_accelerate = self.slow_accelerate + 0.05
                    self.slow_flag = True
                elif (self.slow_flag == True):
                    self.front = self.slow_memset - self.slow_accelerate
                    self.slow_accelerate = self.slow_accelerate + 0.05
                    if (self.front < self.slowdown):
                        self.front = self.slowdown
            elif ((self.stm_warn[0] == "0")or(self.goal_check > self.distance_to_goal)):
                self.slow_accelerate = 0.05
                self.slow_flag = False

            # ===========warning stop============
            if ((self.stm_warn[1] == "1")or(self.stm_warn[3] == "1")or(self.run_button == 1)):
                self.front = 0
            # ===========error wifi============
            #if (self.wifi_msg == 1):
            #    self.front = 0
            # ===========error stop============
            for i in range (0,3):
                if(self.stm_error == "1"):
                    self.front = 0

        ########################################################
        #rospy.loginfo("-D- Traction: %0.2f Rotation: %0.2f" % (self.front,self.angle))
        self.pub_fmotor.publish(self.front)
        self.pub_smotor.publish(self.angle)
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
    #def obstaclesCallback(self,msg):
    #    self.stop_flag = msg.data
    def stopRadCallback(self,msg):
        self.stop_rad = msg.data
    def manualCallback(self,msg):
        self.manual_flag = msg.data
    def stmErrorCallback(self,msg):
        self.stm_error = msg.data
        #print(self.stm_error)
    def stmWarnCallback(self,msg):
        self.stm_warn = msg.data
        #print(self.stm_warn)
    def serverCallback(self,msg):
        self.server_speed = msg.data	
    def wifiCallback(self,msg):
        self.wifi_msg = msg.data
    def runbuttonCallback(self,msg):
        self.run_button = msg.data
    def goalcheckCallback(self,msg):
        self.goal_check = msg.data
	
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
