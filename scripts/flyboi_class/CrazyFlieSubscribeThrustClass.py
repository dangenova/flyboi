#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from mav_msgs.msg import RollPitchYawrateThrust
import math

class SubscribeThrustClassCrazyFlie():
    def __init__(self): 
        #get params
        self.subscribe_topic_name= rospy.get_param("mpc_command_subscribe_topic_name", "/command/roll_pitch_yawrate_thrust")
        self.publish_topic_name = rospy.get_param("~publish_topic_name", "cmd_vel" )
        self.ThrustRate = rospy.get_param("~ThrustRateConstant", .000943) 
        self.ThrustOffset = rospy.get_param("~ThrustOffsetConstant", -5.537574)
        self.Newton2GramForceConstant = rospy.get_param("~Newton2GramForceConstant", .0098066)
        
        #setup subscriber and publisher
        self.subscribe = rospy.Subscriber(self.subscribe_topic_name, RollPitchYawrateThrust, self.callback)
        self.publisher = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=1)
        self.publisher_stamp = rospy.Publisher('/crazyflie/cmd_vel_stamp', TwistStamped, queue_size=1)
        
        # Initialze Message
        self.command_msg = Twist()
        self.command_msg_stamp = TwistStamped()
        
        #Various Flags
        self.counter = 0
        self.overflow_counter = 0
        self.first_callback_flag = 1

        #Constants
        self.rad2deg = 180/math.pi
        self.Thrust2PWM = 2**16/60.0
        self.Athrust = 6.0912*10**-9
        self.Bthrust = 5.4351*10**-4
        self.Cthrust = .323329

        rospy.loginfo('Crazyflie MPC Command Conversion Initialized')
        
    def callback(self,data):
        if self.first_callback_flag == 1:
            rospy.loginfo('Crazyflie Wrapper Receives first MPC Command')
            self.first_callback_flag = 0
        
        # Crazyflie driver will not work unless there is an initial thrust value of zero
        if self.counter < 2:
            self.command_msg.linear.y = 0
            self.command_msg.linear.x = 0
            self.command_msg.angular.z = 0
            self.command_msg.linear.z =  0

            self.command_msg_stamp.twist.linear = self.command_msg.linear
            self.command_msg_stamp.twist.angular = self.command_msg.angular

            self.counter +=1
        else:
            #Basic Conversion Crazyflie input in Deg
            self.command_msg.linear.y = data.roll*self.rad2deg
            self.command_msg.linear.x = data.pitch*self.rad2deg
            self.command_msg.angular.z = data.yaw_rate*self.rad2deg
            self.command_msg.linear.z =  self.ThrustToPWMConversion(data.thrust.z)

            self.command_msg_stamp.twist.linear = self.command_msg.linear
            self.command_msg_stamp.twist.angular = self.command_msg.angular
            
    def ThrustToPWMConversion(self, thrust_force_newton):
        thrust_force_gram = thrust_force_newton/self.Newton2GramForceConstant

        #PWM = 2**16/60.0*thrust_force_gram.     
        #Given PWM = a*PWM^2 +b*PWM +c
        D = self.Bthrust**2-(4*self.Athrust*(self.Cthrust-thrust_force_gram))

        if D < 0:
            D = 0
        PWM = (-self.Bthrust+math.sqrt(D))/(2*self.Athrust) 


        if PWM > 52420:
            PWM = 52420
        if PWM < 16390:
            PWM = 16390
            
        # if self.overflow_counter == 0:
        #     rospy.loginfo(PWM)
        #     rospy.loginfo('Thrust in gf %s' %thrust_force_gram)
        #     rospy.loginfo('roll %s pitch %s', self.command_msg.linear.y, self.command_msg.linear.x)
        #     rospy.loginfo('Thrust in  N %s'%thrust_force_newton)
        #     self.overflow_counter = 50
        
        # self.overflow_counter -= 1

        return PWM
    
    def Publish(self):
        self.command_msg_stamp.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command_msg)
        self.publisher_stamp.publish(self.command_msg_stamp)

    def ReadRollPitchYawrate(self):
        roll = self.command_msg.linear.y
        pitch = self.command_msg.linear.x
        yawrate = self.command_msg.angular.z
        
        return roll, pitch, yawrate

    def Read(self):
        roll = self.command_msg.linear.y
        pitch = self.command_msg.linear.x
        yawrate = self.command_msg.angular.z
        thrust = self.command_msg.linear.z
        
        return roll, pitch, yawrate, thrust



