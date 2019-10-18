#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_msgs.msg
from nav_msgs.msg import Odometry



class Path():
	def __init__(self,z_o, t_o, roll_o, pitch_o, yaw_o):
		self.x_o = 0
		self.y_o = 0
		self.z_o = z_o
		self.t_o = t_o
		self.t = 0
		self.t_1 = 0
		self.roll_o = roll_o
		self.pitch_o = pitch_o
		self.yaw_o = yaw_o
		self.quaternion = [0, 0, 0, 0]

		self.msg = PoseStamped()
		self.header = std_msgs.msg.Header()

		self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
		self.msg.pose.orientation.x = self.quaternion[0]
		self.msg.pose.orientation.y = self.quaternion[1]
		self.msg.pose.orientation.z = self.quaternion[2]
		self.msg.pose.orientation.w = self.quaternion[3]
		self.print_flag = 0

		self.subscribe = rospy.Subscriber('/crazyflie/odom', Odometry , self.callback)
		self.pos_pub = rospy.Publisher('/command/pose', PoseStamped ,queue_size=10)

		self.first_run_flag = 0
	
	def callback(self, data):
		if self.first_run_flag == 0:
			self.x_o = data.pose.pose.position.x
			self.y_o = data.pose.pose.position.y
			self.first_run_flag +=1

		self.t = rospy.get_time()
		self.pos_pub.publish(path.update())

		


	def update(self):
		self.msg.header.stamp = rospy.Time.now()
		self.msg.pose.position.x = self.x_o
		self.msg.pose.position.y = self.y_o
		self.msg.pose.position.z = 1

		
		# if self.t-self.t_o < 2:
		# 	self.msg.pose.position.z = 0
		# 	# self.msg.pose.position.z = 0.0
		# elif (self.t-self.t_o) >= 2 and (self.t-self.t_o)<15:
		# 	if self.print_flag == 0:
		# 		rospy.loginfo('Publishing Hover Position')
		# 		self.print_flag = 1
		# 	self.msg.pose.position.z = self.z_o
		# else:
		# 	if self.print_flag == 1:
		# 		rospy.loginfo('Descending')
		# 		self.print_flag = 2
		# 		self.t_1 = rospy.get_time()
		# 	self.msg.pose.position.z = self.z_o*-.1*(self.t-self.t_1)
		# 	if self.msg.pose.position.z <= 0:
		# 		self.msg.pose.position.z = 0

		# rospy.loginfo('the time is %.2f',t-t_o)
		return self.msg

if __name__ == '__main__':
	rospy.init_node('Position_Reference_Node', anonymous=False)
	
	t_o = rospy.get_time()
	x_o = 0.0
	y_o = 0.0
	z_o = 1.0
	roll_o = 0
	pitch_o = 0
	yaw_o = 0

	hz = rospy.get_param('/flyboi/sampling_frequency', 50)
	r = rospy.Rate(hz)  #time in hz
	rospy.loginfo('Inititilize Reference Node')

	path = Path(z_o, t_o, roll_o, pitch_o, yaw_o)

	while not rospy.is_shutdown():
		rospy.spin()
