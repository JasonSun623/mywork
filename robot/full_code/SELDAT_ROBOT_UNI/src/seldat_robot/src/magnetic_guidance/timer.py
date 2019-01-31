#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 10:49:28 2018

@author: dongho
"""

import config

def timer(pos,loss_line_flag):
    if pos >= 4 and pos < 6:
        if loss_line_flag == 1:
            config.time = -590#1.4
        else:
            config.time = -190#0.9
    elif pos >= 6 and pos < 7:
        config.time = -160#0.6
    elif pos < 4 :
        if loss_line_flag == 1:
            config.time = -570#1.4
        else:
            config.time = -470#1.4
    elif pos > 9 and pos <= 12:
        config.time = -160#0.6
    elif pos > 12 and pos <= 14:
        if loss_line_flag == 1:
            config.time = -590#1.4
        else:
            config.time = -190#0.9
    elif pos > 14:
        if loss_line_flag == 1:
            config.time = -570#1.4
        else:
            config.time = -470#1.4
    config.line_pos.publish(pos)