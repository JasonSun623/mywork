#!/usr/bin/env python
"""
  Odom - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
import tf

from math import sin, cos, pi
from std_msgs.msg import Int16,Int32,Int64, Float32, Float64
from numpy import array
from geometry_msgs.msg import Vector3, Quaternion, Twist, Point, Pose ,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import time

######################################################
######################################################
class Odom():

    #####################################################
    def __init__(self):
    #####################################################
	print "Initializing Odom Class..."
        rospy.init_node("Odometry")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
	self.y = 0.0
	self.x = 0.0
	#self.z = 0.0
	#self.w = 0.0
	self.linear = 0.0
	self.angular = 0.0
	self.current_time = 0
	
	self.odom_encoder = Odometry()
	self.odom_combined = PoseWithCovarianceStamped()

        ### get parameters #### 
        self.rate = rospy.get_param('~rate',50) ##1
        
        #### subscribers/publishers 
        rospy.Subscriber("odom_encoder", Odometry, self.odom_encoderCallback) 
        rospy.Subscriber("robot_pose_ekf/odom_combined",PoseWithCovarianceStamped, self.odom_combinedCallback)	
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	self.odom_broadcaster = tf.TransformBroadcaster()
    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate)
	while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()
            
    #####################################################
    def spinOnce(self):
    #####################################################
        while not rospy.is_shutdown():
            self.calcOdom()
	    self.r.sleep()   
    #####################################################
    def calcOdom(self):
    #####################################################
	self.current_time = rospy.Time.now()

	self.x = self.odom_combined.pose.pose.position.x
	self.y = self.odom_combined.pose.pose.position.y
	#self.z = self.odom_conbined.pose.pose.orientation.z
	#self.w = self.odom_combined.pose.pose.orientation.w
	#self.z = self.odom_combined.pose.pose.orientatuin.z

	quaternion = Quaternion()
	quaternion.x = 0.0
	quaternion.y = 0.0
	quaternion.z = self.odom_combined.pose.pose.orientation.z
	quaternion.w = self.odom_combined.pose.pose.orientation.w

    	self.odom_broadcaster.sendTransform(
        	(self.x, self.y, 0.0),
        	(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        	self.current_time,
        	"base_link",
        	"odom"
    	)

    	odom = Odometry()
    	odom.header.stamp = self.current_time
    	odom.header.frame_id = "odom"

	odom.pose.pose.position.x = self.x
	odom.pose.pose.position.y = self.y
	odom.pose.pose.position.z = 0
	odom.pose.pose.orientation = quaternion
	
    	odom.child_frame_id = "base_link"
	self.linear = self.odom_encoder.twist.twist.linear.x
	self.angular = self.odom_encoder.twist.twist.angular.z

    	odom.twist.twist = Twist(Vector3(self.linear, 0, 0), Vector3(0, 0, self.angular))

        odom.pose.covariance = self.odom_combined.pose.covariance
        
        odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
	
        self.odom_pub.publish(odom)
        
    #####################################################
    def odom_encoderCallback(self, msg):
	self.odom_encoder = msg
    ######################################################

    #####################################################
    def odom_combinedCallback(self, msg):
	self.odom_combined = msg
	#self.z = msg.pose.pose.orientation.z
	#print(self.odom_combined)
    ######################################################


if __name__ == '__main__':
    """ main """
    odo = Odom()
    odo.spin()   
