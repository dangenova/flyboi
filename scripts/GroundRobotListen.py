#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
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


        self.HeightOffset= rospy.get_param('~HeightOffset', 1)
        self.publish_topic_name= rospy.get_param('~PublishTopic', "/command/pose")
        self.timer_subscribe_topic_name= rospy.get_param('~TimerSubscribeTopic', "/crazyflie/odom_mpc")
        self.reference_subscribe_topic_name= rospy.get_param('~ReferenceSubscribeTopic', "/vrpn_client_node/robot1/pose")

        rospy.loginfo('Height Offset is %s', self.HeightOffset)
        rospy.loginfo('Publish Topic is is %s', self.publish_topic_name)
        rospy.loginfo('Timer Topic is is %s', self.timer_subscribe_topic_name)
        rospy.loginfo('Reference Topic is is %s', self.reference_subscribe_topic_name)

        #self.timer_subscribe = rospy.Subscriber(self.timer_subscribe_topic_name, Odometry , self.timer_callback)
        rospy.Timer(rospy.Duration(.02), self.timer_callback)
        self.reference_subscribe = rospy.Subscriber(self.reference_subscribe_topic_name, PoseStamped , self.reference_callback)
        self.pos_pub = rospy.Publisher(self.publish_topic_name, PoseStamped ,queue_size=10)
    
    def timer_callback(self, data):
        
        self.msg.header.stamp = rospy.Time.now()
        self.pos_pub.publish(self.msg)

    def reference_callback(self,data):
        self.msg.pose.position.x = data.pose.position.x
        self.msg.pose.position.y = data.pose.position.y
        self.msg.pose.position.z = data.pose.position.z + self.HeightOffset



if __name__ == '__main__':
    rospy.init_node('GroundRobotPositionNode', anonymous=False)
    rospy.loginfo('Ground Robot Position Node Initialized')

    path = Path()

    while not rospy.is_shutdown():
        rospy.spin()