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