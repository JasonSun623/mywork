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
from geometry_msgs.msg import Vector3, Quaternion, Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import time

######################################################
######################################################
class Odom():

    #####################################################
    def __init__(self):
    #####################################################
	print "Initializing Odom Encoder Class..."
        rospy.init_node("Odometry_Encoder")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
        self.motor = 0
        self.vel = 0
	self.instant_vel = 0
	
	self.v_front = 0
	self.w_front = 0
	self.delta_distance = 0
	self.delta_th = 0
	self.delta_x = 0
	self.delta_y = 0
	self.x = 0.0
	self.y = 0.0
	self.th = 0.0
	self.vel_robot = 0
	self.rad_robot = 0
	self.current_time = 0

        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0
	self.rotation_encoder = 0

        ### get parameters #### 
        self.rate = rospy.get_param('~rate',50) ##1
        self.rolling_pts = rospy.get_param('~rolling_pts',2)
	#xung tren met: (64 xung, gear 1:34, duong kinh 250mm) = (64*34*1000)/(250*3.14)
        self.ticks_per_meter = rospy.get_param('ticks_meter', 2771)
	self.RadianPerCount = rospy.get_param('radian_per_count',0.00062832)#0.001533)#9808)
	self.vel_threshold = rospy.get_param('~vel_threshold', 0.001) #0.05)
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max',  2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        
        #### subscribers/publishers 
        rospy.Subscriber("pos", Vector3, self.wheelCallback) 
	self.odom_pub = rospy.Publisher("odom_encoder", Odometry, queue_size=50)
	self.odom_broadcaster = tf.TransformBroadcaster()
    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate) 
	self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()

        while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()
            
    #####################################################
    def spinOnce(self):
    #####################################################
        self.prev_vel = [0.0] * self.rolling_pts
        self.vel = 0.0

        while not rospy.is_shutdown():
            self.calcOdom()
	    self.r.sleep()   
    #####################################################
    def calcOdom(self):
    #####################################################
	self.current_time = rospy.Time.now()
        self.dt_duration = self.current_time - self.then
        self.dt = self.dt_duration.to_sec()
	
        if (self.wheel_latest == self.wheel_prev):
            
	    cur_vel = (1 / self.ticks_per_meter) / self.dt
            if abs(cur_vel) < self.vel_threshold: 
                rospy.logdebug("-D- %s below threshold cur_vel=%0.3f vel=0" % (self.nodename, cur_vel))
                self.appendVel(0)
                self.calcRollingVel()
            else:
                rospy.logdebug("-D- %s above threshold cur_vel=%0.3f" % (self.nodename, cur_vel))
                if abs(cur_vel) < self.vel:
                    rospy.logdebug("-D- %s cur_vel < self.vel" % self.nodename)
                    self.appendVel(cur_vel)
                    self.calcRollingVel()
	    self.instant_vel = 0
	    
	    #cur_vel = 0
	    #self.appendVel(cur_vel)
	    #self.calcRollingVel()
	    
	    #self.instant_vel = 0
	    #self.vel = 0
        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev)/self.dt
	    self.appendVel(cur_vel)
            self.calcRollingVel()
	    #print (self.wheel_prev)
	    #print (self.wheel_latest)
	    #print (self.dt)
	    #print ("====================================")
            self.wheel_prev = self.wheel_latest
            self.then = rospy.Time.now()
	    self.instant_vel = cur_vel
	

	self.v_front = self.instant_vel #front velocity
	self.w_front = self.rotation_encoder * self.RadianPerCount
	
	self.delta_distance = self.instant_vel * self.dt
	self.delta_th = ((self.v_front*sin(self.w_front))/1.0)*self.dt
	self.delta_x = self.delta_distance*cos(self.w_front)*cos(self.th)
	self.delta_y = self.delta_distance*cos(self.w_front)*sin(self.th)

	self.x += self.delta_x
	self.y += self.delta_y
	self.th += self.delta_th
	
	if (self.th > pi):
		self.th -= (2*pi)
	elif (self.th <= -pi):
		self.th += (2*pi)
 
	self.vel_robot = self.vel * cos(self.w_front) #v_front
	self.rad_robot = sin(self.w_front) * self.vel / 1.0

	#print (self.vel_robot)
	#print (self.rad_robot)
    	odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
    	self.odom_broadcaster.sendTransform(
        	(self.x, self.y, 0.0),
        	odom_quat,
        	self.current_time,
        	"base_footprint",
        	"odom_encoder"
    	)

    	odom = Odometry()
    	odom.header.stamp = self.current_time
    	odom.header.frame_id = "odom_encoder"

    	odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
	
    	odom.child_frame_id = "base_footprint"
    	odom.twist.twist = Twist(Vector3(self.vel_robot, 0, 0), Vector3(0, 0, self.rad_robot))

        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
                
        odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
	
        self.odom_pub.publish(odom)
        
    #####################################################
    def appendVel(self, val):
    #####################################################
        self.prev_vel.append(val)
        del self.prev_vel[0]
        
    #####################################################
    def calcRollingVel(self):
    #####################################################
        p = array(self.prev_vel)
        self.vel = p.mean()
 
    #####################################################
    def wheelCallback(self, msg):
    ######################################################
        enc = msg.x
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1    
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
	self.prev_encoder = enc
    	self.rotation_encoder = msg.z

if __name__ == '__main__':
    """ main """
    odo = Odom()
    odo.spin()   
