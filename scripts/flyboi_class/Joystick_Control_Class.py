#!/usr/bin/env python
# Mapping joystick output to /crazyflie/cmd_vel
# Author: Peng Wei

import rospy

#from crazyflie_driver.srv import UpdateParams
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
from flyboi_ethz.msg import Joyfilter


MAX_THROTTLE = 65000
MIN_THROTTLE = 36000
ARM_THROTTLE = 20000
DEADZONE     = 0.2

class Joystick():
	def __init__(self):
	# 	self.name = rospy.get_param("~joy_topic", '/joy')
	# 	self.Max_roll   = rospy.get_param("~Max_roll_angle", 30.0)
	# 	self.Max_pitch  = rospy.get_param("~Max_pitch_angle", 30.0)
	# 	self.Max_yaw    = rospy.get_param("~Max_yaw_rate", 120.0)
	# 	self.Max_thrust = rospy.get_param("~Max_thrust", 0.9)

		self.name = rospy.get_param("~joy_topic", '/joy')
		self.Max_roll   =  25.0
		self.Max_pitch  = 25.0
		self.Max_yaw    = 120.0
		self.Max_thrust =  0.9
		self.twist = Twist()
		self.joy = Joyfilter()
		self.joy.axes0 = -0.0
		self.joy.axes1 = -0.0
		self.joy.axes2 = -0.0
		self.joy.axes3 = -0.1 + DEADZONE
		self.status = "Idle"
		self.firstcount = 1

		rospy.Subscriber(self.name, Joy, self.mapping)
	
	def joy_reset(self):
		self.joy.axes0 = -0.0
		self.joy.axes1 = -0.0
		self.joy.axes2 = -0.0
		self.joy.axes3 = -1 + DEADZONE

	def filter_joy(self, data):
		a0 = 0.6
		a1 = 0.6
		a2 = 0.2
		a3 = 0.6
		self.joy.axes0 = a0*data.axes[0] + (1-a0)*self.joy.axes0
		self.joy.axes1 = a1*data.axes[1] + (1-a1)*self.joy.axes1
		self.joy.axes2 = a2*data.axes[2] + (1-a2)*self.joy.axes2
		self.joy.axes3 = a3*data.axes[3] + (1-a3)*self.joy.axes3
		return self.joy	

	def linear_map(self, x, r1_min, r1_max, r2_min, r2_max):
		return (x-r1_min)/(r1_max-r1_min)*(r2_max-r2_min)+r2_min

	def mapping(self, data):
		# roll, pitch, yaw rate and throttle mapping	
		if self.status == "Idle":
			self.twist = Twist()
			self.status = "Armed"

		elif self.status == "Armed":
			if data.axes[3] < -1+DEADZONE or self.firstcount == 1:
				self.twist.linear.y = 0
				self.twist.linear.x = 0
				self.twist.linear.z = ARM_THROTTLE
				self.twist.angular.z = 0	
				self.joy_reset()
				self.firstcount = 0
			else:
				self.filter_joy(data)
				self.twist.linear.y  = -1*self.linear_map(self.joy.axes0,-1, 1, -self.Max_roll, self.Max_roll) 
				self.twist.linear.x  =  1*self.linear_map(self.joy.axes1,-1, 1, -self.Max_pitch, self.Max_pitch)
				self.twist.angular.z = -1*self.linear_map(self.joy.axes2,-1, 1, -self.Max_yaw, self.Max_yaw) 
				self.twist.linear.z = self.linear_map(self.joy.axes3, -1+DEADZONE, 1, MIN_THROTTLE, MAX_THROTTLE*self.Max_thrust)

