#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    PID - takes messages on wheel_vtarget 
    Author: Jon Stephan
    Modifier: Robot Team	
"""

import rospy
import roslib

from math import sin, cos, pi
from std_msgs.msg import Int16,Int32,Int64, Float32, Float64
from numpy import array
from geometry_msgs.msg import Vector3 #, Quaternion, Twist
#from nav_msgs.msg import Odometry
#from tf.broadcaster import TransformBroadcaster
import time

######################################################
######################################################
class PidVelocity():

    #####################################################
    def __init__(self):
    #####################################################
	print "Initializing TPID Class..."
        rospy.init_node("t_pid_velocity")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.previous_error = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0

        ### get parameters #### 
        self.Kp = rospy.get_param('~Kp',9000) #4500
        self.Ki_H = rospy.get_param('~Ki_H',300) 	#0=>0.8 130| 0.9=>1.5 200 |1.6=>1.9 280
        self.Kd = rospy.get_param('~Kd',2000) #2000
        
        self.Ki_M = rospy.get_param('~Ki_M',200)
        self.Ki_L = rospy.get_param('~Ki_L',130) #130
        
        #self.Kp_H = rospy.get_param('~Kp_H',35000)
        #self.Ki_H = rospy.get_param('~Ki_H',60)
        #self.Kd_H = rospy.get_param('~Kd_H',1000)

        self.out_min = rospy.get_param('~out_min',-100)
        self.out_max = rospy.get_param('~out_max',4095)

        self.rate = rospy.get_param('~rate',20) ##20
        self.rolling_pts = rospy.get_param('~rolling_pts',2)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4) #4
        #xung tren met: (64 xung, gear 1:34, duong kinh 250mm) = (64*34*1000)/(250*3.14)
        self.ticks_per_meter = rospy.get_param('ticks_meter', 1522)#2770.56925)
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001) #0.05)
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max',  2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = rospy.Time.now()

        #rospy.logdebug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" % (self.nodename, self.Kp, self.Ki, self.Kd, self.ticks_per_meter))
        
        #### subscribers/publishers 
        rospy.Subscriber("pos", Vector3, self.wheelCallback)
        rospy.Subscriber("fwheel_vtarget", Float32, self.targetCallback) 
        
        self.pub_motor = rospy.Publisher('twheel',Int32,queue_size=10)
        #self.= rospy.Publisher('t_vel', Float32,queue_size=10)

    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        #rospy.loginfo("Time.now [%f]", self.then)
        self.ticks_since_target = self.timeout_ticks
        #self.stop = 0
        #rospy.loginfo("wheel_prev [%f]", self.wheel_prev)
        #rospy.loginfo("wheel_latest [%f]", self.wheel_latest)
        self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()
        while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()
            
    #####################################################
    def spinOnce(self):
    #####################################################
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0 
        self.vel = 0.0
        self.prev_pid_time = rospy.Time.now()

        # only do the loop if we've recently recieved a target velocity message
        while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
            self.calcVelocity()
            self.doPid()
            ##########################
            if (self.target < 0):
                self.motor = self.motor * -1
            ##########################
            self.pub_motor.publish(self.motor)
            ##rospy.loginfo("pub_rmotor")
            self.r.sleep()
            self.ticks_since_target += 1

            if (self.ticks_since_target == self.timeout_ticks):#and(self.stop == 0):
            	self.pub_motor.publish(0)
            
    #####################################################
    def calcVelocity(self):
    #####################################################
        self.dt_duration = rospy.Time.now() - self.then
        self.dt = self.dt_duration.to_sec()
        ##rospy.loginfo("==========RRRself.dt = %0.3f" %(self.dt))
        #rospy.loginfo("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" % (self.nodename, self.dt, self.wheel_latest, self.wheel_prev))
        
        if (self.wheel_latest == self.wheel_prev):
            # we haven't received an updated wheel lately
            cur_vel = (1 / self.ticks_per_meter) / self.dt    # if we got a tick right now, this would be the velocity
            if abs(cur_vel) < self.vel_threshold: 
                # if the velocity is < threshold, consider our velocity 0
                rospy.logdebug("-D- %s below threshold cur_vel=%0.3f vel=0" % (self.nodename, cur_vel))
                self.appendVel(0)
                self.calcRollingVel()
            else:
                rospy.logdebug("-D- %s above threshold cur_vel=%0.3f" % (self.nodename, cur_vel))
                if abs(cur_vel) < self.vel:
                    rospy.logdebug("-D- %s cur_vel < self.vel" % self.nodename)
                    # we know we're slower than what we're currently publishing as a velocity
                    self.appendVel(cur_vel)
                    self.calcRollingVel()
            
        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            if (cur_vel > 2.5) or (cur_vel < -2.5):
                cur_vel = 0.0
            self.appendVel(cur_vel)
            self.calcRollingVel()
            #rospy.loginfo("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            #rospy.logdebug("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            self.wheel_prev = self.wheel_latest
            self.then = rospy.Time.now()
            
        #self.pub_vel.publish(self.vel)
        
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
    def doPid(self):
    #####################################################
        self.pid_dt_duration = rospy.Time.now() - self.prev_pid_time
        self.prev_pid_time = rospy.Time.now()
        pid_dt = self.pid_dt_duration.to_sec()
        

        self.error = abs(self.target) - abs(self.vel)
        ###############################
	
        self.integral = self.integral + (self.error) #* pid_dt)
        ##rospy.loginfo("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)" % (self.integral, self.integral, self.error, pid_dt))
        self.derivative = (self.error - self.previous_error)#/ pid_dt
        self.previous_error = self.error

        if (abs(self.target) <= 0.9):
            self.motor = (self.Kp * self.error) + (self.Ki_L * self.integral) + (self.Kd * self.derivative)
            #print('Ki_L')
        elif(abs(self.target) <= 1.6)and(abs(self.target) > 0.9):
            self.motor = (self.Kp * self.error) + (self.Ki_M * self.integral) + (self.Kd * self.derivative)
            #print('Ki_M')
        elif(abs(self.target) > 1.6):
            self.motor = (self.Kp * self.error) + (self.Ki_H * self.integral) + (self.Kd * self.derivative)
            #print('Ki_L')

        if self.motor > self.out_max:
            self.motor = self.out_max
            self.integral = self.integral - (self.error) #* pid_dt)
        if self.motor < self.out_min:
            self.motor = self.out_min
            self.integral = self.integral - (self.error) #* pid_dt)
      
        if (self.target == 0):
            self.motor = 0
	
        rospy.loginfo("Traction-motor vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f motor:%d time:%f" % (self.vel, self.target, self.error, self.integral, self.derivative, self.motor, pid_dt))
        #print "Traction-motor vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f motor:%d time:%f" % (self.vel, self.target, self.error, self.integral, self.derivative, self.motor, pid_dt) #motor
        #print "%0.2f" % (self.vel) #motor

 
    #####################################################
    def wheelCallback(self, msg):
    ######################################################
        enc = msg.x ### msg.x : velocity, msg.z: theta
        ##rospy.loginfo("Traction encoder:  %0.2f)" % (enc))	#######	print encoder
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
           
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.prev_encoder = enc
        
        #rospy.loginfo("-D- %s wheelCallback msg.data= %0.3f wheel_latest = %0.3f mult=%0.3f" % (self.nodename, enc, self.wheel_latest, self.wheel_mult))
    
    ######################################################
    def targetCallback(self, msg):
    ######################################################
        self.target = msg.data
        self.ticks_since_target = 0
        rospy.logdebug("-D- %s targetCallback " % (self.nodename))

    
if __name__ == '__main__':
    """ main """
    pidVelocity = PidVelocity()
    pidVelocity.spin()   
