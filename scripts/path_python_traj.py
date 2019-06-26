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



class Path():
	def __init__(self, x_o, y_o, z_o, roll_o, pitch_o, yaw_o):
		self.x_o = x_o
		self.y_o = y_o
		self.z_o = z_o
		self.t = 0
		self.roll_o = roll_o
		self.pitch_o = pitch_o
		self.yaw_o = yaw_o
		self.quaternion = [0, 0, 0, 0]
		self.msg = PoseStamped()
		self.header = std_msgs.msg.Header()
		self.t_o = 0
		

	def update_pose(self):
		self.msg.header.stamp = rospy.Time.now()
		self.msg.pose.position.x = 5*math.sin(.3*self.t-self.t_o)+self.x_o
		self.msg.pose.position.y = .2*self.y_o*(self.t-self.t_o)
		self.msg.pose.position.z = self.z_o
		self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
		self.msg.pose.orientation.x = self.quaternion[0]
		self.msg.pose.orientation.y = self.quaternion[1]
		self.msg.pose.orientation.z = self.quaternion[2]
		self.msg.pose.orientation.w = self.quaternion[3]
		#rospy.loginfo('the time is %.2f',t-t_o)
		return self.msg

	def update_start_time(self):
		self.t_o = rospy.get_time()

if __name__ == '__main__':
	rospy.init_node('path_python', anonymous=False)
	pos_pub = rospy.Publisher('/target/position', PoseStamped ,queue_size=1)
	x_o = 1.0
	y_o = 1.0
	z_o = 5.0
	roll_o = 0
	pitch_o = 0
	yaw_o = 0

	hz = rospy.get_param('/flyboi/sampling_frequency')
	r = rospy.Rate(hz)  #time in hz
	rospy.loginfo('Trajectory Node Initialized')

	path = Path(x_o, y_o, z_o, roll_o, pitch_o, yaw_o)
	path.update_start_time()
	
	while not rospy.is_shutdown():
		t = rospy.get_time()
		path.t = t
		pos_pub.publish(path.update_pose())
		r.sleep()
