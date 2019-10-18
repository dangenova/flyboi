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
	def __init__(self, x_o, y_o, z_o, t_o, roll_o, pitch_o, yaw_o):
		self.x_o = x_o
		self.y_o = y_o
		self.z_o = z_o
		self.t_o = t_o
		self.t = 0
		self.roll_o = roll_o
		self.pitch_o = pitch_o
		self.yaw_o = yaw_o
		self.quaternion = [0, 0, 0, 0]

		self.msg = PoseStamped()
		self.header = std_msgs.msg.Header()
		
		self.msg_cov = PoseWithCovarianceStamped()
		self.cov_matrix_pub = np.zeros(36)
		cov_matrix = np.identity(6)*.01 
		self.cov_matrix_pub = cov_matrix.flatten()
	#	rospy.Subscriber('/clock', Clock, self.updatetime)

	def update(self):
		self.msg.header.stamp = rospy.Time.now()
		self.msg.pose.position.x = 5*math.cos(.4*self.t-self.t_o)+self.x_o
		self.msg.pose.position.y = .2*self.y_o*(self.t-self.t_o)
		self.msg.pose.position.z = self.z_o
		self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
		self.msg.pose.orientation.x = self.quaternion[0]
		self.msg.pose.orientation.y = self.quaternion[1]
		self.msg.pose.orientation.z = self.quaternion[2]
		self.msg.pose.orientation.w = self.quaternion[3]
		rospy.loginfo('the time is %.2f',t-t_o)
		return self.msg

	def cov(self):
		self.msg_cov.pose.pose.position.x = 5*math.cos(.4*self.t-self.t_o)+self.x_o
		self.msg_cov.pose.pose.position.y = .2*self.y_o*(self.t-self.t_o)
		self.msg_cov.pose.pose.position.z = self.z_o
		self.msg_cov.pose.pose.orientation.x = self.quaternion[0]
		self.msg_cov.pose.pose.orientation.y = self.quaternion[1]
		self.msg_cov.pose.pose.orientation.z = self.quaternion[2]
		self.msg_cov.pose.pose.orientation.w = self.quaternion[3]
		self.msg_cov.pose.covariance = self.cov_matrix_pub
	
		return self.msg_cov

	#def updatetime(self,data):
	#	self.t = data.to_sec()


if __name__ == '__main__':
	rospy.init_node('path_python', anonymous=False)
	publish_topic_name= rospy.get_param('~PublishTopic', "/command/pose")
	pos_pub = rospy.Publisher(publish_topic_name, PoseStamped ,queue_size=10)
	t_o = rospy.get_time()
	x_o = 1.0
	y_o = 1.0
	z_o = 5.0
	roll_o = 0
	pitch_o = 0
	yaw_o = 0

	r = rospy.Rate(50)  #time in hz
	rospy.loginfo('I exist')

	path = Path(x_o, y_o, z_o, t_o, roll_o, pitch_o, yaw_o)
	while not rospy.is_shutdown():
		t = rospy.get_time()
		path.t = t
		pos_pub.publish(path.update())
		#cov_pub.publish(path.cov())
		#rospy.loginfo('update') 
		r.sleep()
