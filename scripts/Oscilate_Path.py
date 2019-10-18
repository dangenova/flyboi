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
	def __init__(self):
		self.x_o = 0
		self.y_o = 0
		self.z_o = 0
		self.t = 0
		self.roll_o = 0
		self.pitch_o = 0
		self.yaw_o = 0
		self.quaternion = [0, 0, 0, 0]

		self.msg = PoseStamped()
		self.header = std_msgs.msg.Header()

		self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
		self.msg.pose.orientation.x = self.quaternion[0]
		self.msg.pose.orientation.y = self.quaternion[1]
		self.msg.pose.orientation.z = self.quaternion[2]
		self.msg.pose.orientation.w = self.quaternion[3]
		self.print_flag = 0

		self.publish_topic_name= rospy.get_param('~PublishTopic', "/command/pose")
		self.subscribe_topic_name= rospy.get_param('~SubscribeTopic', "/crazyflie/odom_mpc")
		self.subscribe = rospy.Subscriber(self.subscribe_topic_name , Odometry , self.callback)
		self.pos_pub = rospy.Publisher(self.publish_topic_name, PoseStamped ,queue_size=10)

		self.first_run_flag = 0
	
	def callback(self, data):
		if self.first_run_flag == 0:
			self.x_o = data.pose.pose.position.x
			self.y_o = data.pose.pose.position.y
			self.z_o = data.pose.pose.position.z
			self.first_run_flag +=1
			self.t_o = rospy.get_time()

		self.t = rospy.get_time()
		self.pos_pub.publish(path.update())

		


	def update(self):
		t = self.t - self.t_o
		self.msg.header.stamp = rospy.Time.now()
		
		self.msg.pose.position.y = self.y_o
		self.msg.pose.position.z = self.z_o
		self.msg.pose.position.x = self.x_o +0.5*math.sin(t)
		return self.msg

if __name__ == '__main__':
	rospy.init_node('Position_Reference_Node', anonymous=False)
	rospy.loginfo('Inititilize Reference Node')

	path = Path()

	while not rospy.is_shutdown():
		rospy.spin()
