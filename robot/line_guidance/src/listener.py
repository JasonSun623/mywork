#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 19:48:41 2019

@author: dongho
"""

import rospy
from std_msgs.msg import String
#import os

#import logging

#logging.basicConfig(
#    filename="log_line/log.txt",
#    level=logging.INFO,
#    format="%(asctime)s:%(levelname)s:%(message)s"
#    )

#import logging
#logger = logging.getLogger('myapp')
#hdlr = logging.FileHandler('log_line/log.txt')
#formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
#hdlr.setFormatter(formatter)
#logger.addHandler(hdlr) 
#logger.setLevel(logging.INFO)
#def logfile(filename,data):
#    if not os.path.exists(os.path.dirname(filename)):
#        try:
#            os.makedirs(os.path.dirname(filename))
#        except OSError as exc: # Guard against race condition
#            if exc.errno != errno.EEXIST:
#                raise
#    
#    with open(filename, "w") as f:
#        f.writelines('%s\n' % data)

class line_follow():

    ##############################____INIT____############################
    
    def __init__(self):
        self,count = 0
        rospy.init_node('LINE_FOLLOWER')
        key_sub = rospy.Subscriber('/key_press', String, self.key_callback)
        
    def key_callback(self,msg):
        data = msg.data
        if data == 'stop pid':
            self.PID_enable = 2
        elif data == 'key p is press' :
            self.PID_enable = 1
        elif data == 'key m is press' :
            self.PID_enable = 2
        elif data == 'key l is press' :
            self.PID_enable = 3
        elif data == 'key j is press' :
            self.PID_enable = 4
        elif data == 'key k is press':
            self.PID_enable = 5
        
if __name__ == '__main__':
    run = line_follow()
    run.main()