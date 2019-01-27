#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 08:12:13 2018

@author: dongho
"""
import config

def position(mag_ss):
        ss_sum = 0
        ss_aveg = 0
        for i in range(0,len(mag_ss)) :
            ss_aveg += mag_ss[i] * i
            ss_sum += int(config.mag_ss[i])
        if ss_sum == 0:
            pos = 8
            config.line_flag = 1
        else:
            pos = (ss_aveg/ss_sum) + 1
            config.line_flag = 0
        if pos < 7:
            config.pos_left = 1
            config.pos_right = 0
            config.center = 0
        elif pos > 9:
            config.pos_left = 0
            config.pos_right = 1
            config.center = 0
        return pos