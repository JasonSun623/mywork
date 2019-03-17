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

import logging
logger = logging.getLogger('myapp')
hdlr = logging.FileHandler('log_line/log.txt')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
hdlr.setFormatter(formatter)
logger.addHandler(hdlr) 
logger.setLevel(logging.INFO)
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
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    logger.info('data %s', data.data)
    #logToFile('/log_line/log.txt',data.data)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()