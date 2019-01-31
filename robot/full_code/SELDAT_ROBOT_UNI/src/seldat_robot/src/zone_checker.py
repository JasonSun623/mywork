#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Check for which zone is robot in
    Author: Robot Team
	
"""

import rospy
import sys
import time
import math
import numpy as np
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3

class zone_checker(object):

 def __init__(self):
  print "Initializing Zone Checker Class..."
  #self.docking_zoneadad  = np.array([[1, 2],[3, 4],[5, 6],[7, 8]])
  #self.docking_zone  = np.array([[-6.3 -4.5],[-19.3 -4.5],[-19.3 4],[-6.3 4]])
  #self.put_away_zone = np.array([[16 5],[16 -3],[3.4 -3],[3.4 5]])
  #self.ready_zone    = np.array([[17 -5.1],[17 -7.1],[11 -7.1],[11 -5.1]])
  #self.highway1      = np.array([[10.7 -5.1],[10.7 -7.1],[-7 -7.1],[-7 -5.1]])
  #self.highway2      = np.array([[1 -0.5],[1 -2.5],[-3.4 -2.5],[-3.4 -0.5]])
  #Check point Docking => Highway2
  #self.checkpoint1 = np.array([[-3.5 -0.5],[-3.5 -2.5],[-6.2 -2.5],[-6.2 -0.5]])
  #Check point Highway2 => Put away
  #self.checkpoint2 = np.array([[3.3 -0.5],[3.3 -2.5],[1.1 -2.5],[1.1 -0.5]])
  #Check point Put away => Ready/Highway1
  #self.checkpoint3 = np.array([[11 -3.1],[11 -5],[9 -5],[9 -3.1]])
  #Check point Highway1 => Docking
  #self.checkpoint4 = np.array([[-7.1 -5.1],[-7.1 -7.1],[-11.5 -7.1],[-11.5 -5.1]])

  #Docking > Putaway > Ready > Highway1 > Highway2 >
  #Docking_Highway2 > Highway2_Putaway > Putaway_Ready/Highway1 > Highway1_Docking
  #self.zoneArea = np.array([[-6.3, -4.5, -19.3, -4.5, -19.3, 4, -6.3, 4], \
  #[16, 5, 16, -3, 3.4, -3, 3.4, 5], [17, -5.1, 17, -7.1, 11, -7.1, 11, -5.1], \
  #[10.7, -5.1, 10.7, -7.1, -7, -7.1, -7, -5.1], [1, -0.5, 1, -2.5, -3.4, -2.5, -3.4, -0.5], \
  #[-3.5, -0.5, -3.5, -2.5, -6.2, -2.5, -6.2, -0.5], [3.3, -0.5, 3.3, -2.5, 1.1, -2.5, 1.1, -0.5], \
  #[11, -3.1, 11, -5, 9, -5, 9, -3.1], [-7.1, -5.1, -7.1, -7.1, -11.5, -7.1, -11.5, -5.1]])

  self.zoneArea = np.array([[-6.3, 4,  -6.3, -3, -19.3, 4], \
  [16, 5, 16, -3, 7.4, 5], [17, -5.1, 17, -7.1, 11, -5.1], \
  [10.7, -5.1, 10.7, -7.1, -7, -5.1], [5.9, -0.5, 5.8, -2.5, -3.4, -0.5], \
  [-3.5, -0.5, -3.5, -2.5, -6.2, -0.5], [7.3, 5, 7.3, 1.2, 3.67, 5], \
  [11, -3.1, 11, -5, 9, -3.1], [-7.1, -3.3, -7.1, -7.1, -11.5, -3.3]])

  self.zoneMsg = String()
  self.amclpose_X = 0
  self.amclpose_Y = 0
  #self.amclpose_theta = 0
  self.zone_pub = rospy.Publisher('zoneChecker',String,queue_size = 10)
  rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amclCallback)

 def zone(self):
  try:
    x_ = self.amclpose_X
    y_ = self.amclpose_Y
    #print(self.zoneArea)
    #print("==========================")
    #theta = atan2(y,x)
    time.sleep(0.5)
    for x in range(0, len(self.zoneArea)):
        p21_x = self.zoneArea[x][2]-self.zoneArea[x][0]
        p21_y = self.zoneArea[x][3]-self.zoneArea[x][1]
        p41_x = self.zoneArea[x][4]-self.zoneArea[x][0]
        p41_y = self.zoneArea[x][5]-self.zoneArea[x][1]
        p21_magnitude = pow(p21_x,2) + pow(p21_y,2)
        p41_magnitude = pow(p41_x,2) + pow(p41_y,2)
        p_x = x_ - self.zoneArea[x][0]
        p_y = y_ - self.zoneArea[x][1]
        condition_1 = (p_x * p21_x) + (p_y * p21_y)
        condition_2 = (p_x * p41_x) + (p_y * p41_y)
        if ((0 <= condition_1)and(condition_1 <= p21_magnitude)):
            if ((0 <= condition_2)and(condition_2 <= p41_magnitude)):
                #print("Inside")
                if (x == 0):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Docking\"}"
                    rospy.loginfo("Docking")
                elif (x == 1):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Put_away\"}"
                    rospy.loginfo("Put away")
                elif (x == 2):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Ready\"}"
                    rospy.loginfo("Ready")
                elif (x == 3):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Highway1\"}"
                    rospy.loginfo("Highway1")
                elif (x == 4):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Highway2\"}"
                    rospy.loginfo("Highway1")
                elif (x == 5):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Docking_Highway2\"}"
                    rospy.loginfo("Docking => Highway2")
                elif (x == 6):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Highway2_Put_away\"}"
                    rospy.loginfo("Highway2 => Put away")
                elif (x == 7):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Put_away_Ready/Highway1\"}"
                    rospy.loginfo("Put away => Highway1")
                elif (x == 8):
                    self.zoneMsg = "{\"robot\": \"Robot_2\", \"zone\":\"Highway1_Docking\"}"
                    rospy.loginfo("Highway1 => Docking")
                if (x_ != 0)and(y_ != 0):
                    self.zone_pub.publish(self.zoneMsg)
  except:
   rospy.logwarn("Error in zone publish")
   pass

 def amclCallback(self, msg):
  self.amclpose_X = msg.pose.pose.position.x
  self.amclpose_Y = msg.pose.pose.position.y
  #print(self.amclpose_X)
  #self.amclpose_X = acos(msg.pose.pose.orientation.w)*2

if __name__=='__main__':
 """ main """
 rospy.init_node('Zone_Checker_Node', anonymous=True)
 zoneC = zone_checker()
 try:
  while not rospy.is_shutdown():
   zoneC.zone()
 except rospy.ROSInterruptException:
  rospy.logwarn("Error in main function")
