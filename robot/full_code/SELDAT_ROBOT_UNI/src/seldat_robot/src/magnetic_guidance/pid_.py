#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 11:19:48 2018

@author: dongho
"""

import config

def pid_cal(pos,kp,kd):
    proportional = pos - config.set_point
    if config.cross_detect == 1:
        proportional = config.last_proportional
    else:
        proportional = proportional
    #self.integral = self.integral + proportional
    derivative = (proportional - config.last_proportional)
    ster_value = int(proportional* kp +  derivative*kd) #- 1
    config.last_proportional = proportional
    ##print "pos = ",pos,"ster_value = ",ster_value
    return ster_value

def angle_controll(pos,speed):
        turning_value = self.pid_cal(self.position(self.mag_ss),12,150)#15,200
        #print "turning_value = ",int(turning_value)
        self.angle = self.home_value + turning_value
        self.angle = round(self.angle)
        self.angle = int(self.angle)
        ##print "self.angle",self.angle
        if self.angle > 950:
            self.angle = 950
        elif self.angle < 5:
            self.angle = 5
        #print "speed",speed, "angle",self.angle
        ##print "left = ", self.left,"right = ",self.right,"center = ",self.center
        self.vel_pub.publish(speed)
        self.ste_pub.publish(self.angle)