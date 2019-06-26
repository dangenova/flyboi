#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import message_filters

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SubscribeAndPublishOdom():
    def __init__(self):
        #Stuff for message sync
        self.pose_sub = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        self.vel_sub = message_filters.Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        self.ts = message_filters.TimeSynchronizer([self.pose_sub, self.vel_sub],2)
        self.ts.registerCallback(self.callback)
        
        #Publish Odometry
        self.odom_pub = rospy.Publisher('/ardu/odom', Odometry, queue_size=10 )
        
        #Initilize Odometry Matrix
        self.odom = Odometry()
        self.cov_matrix_pub = np.zeros(36)
        cov_matrix = np.identity(6)*.01 
        self.cov_matrix_pub = cov_matrix.flatten()
        self.odom.pose.covariance = self.cov_matrix_pub
        self.odom.twist.covariance = self.cov_matrix_pub

    def callback(self, pose, vel):
        self.odom.pose.pose.position.x = pose.pose.position.x
        self.odom.pose.pose.position.y = pose.pose.position.y
        self.odom.pose.pose.position.z = pose.pose.position.z
        self.odom.pose.pose.orientation.x = pose.pose.orientation.x
        self.odom.pose.pose.orientation.y = pose.pose.orientation.y
        self.odom.pose.pose.orientation.z = pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = pose.pose.orientation.w
        

        self.odom.twist.twist.linear.x = vel.twist.linear.x
        self.odom.twist.twist.linear.y = vel.twist.linear.y
        self.odom.twist.twist.linear.z = vel.twist.linear.z
        self.odom.twist.twist.angular.x = vel.twist.angular.x
        self.odom.twist.twist.angular.y = vel.twist.angular.y
        self.odom.twist.twist.angular.z = vel.twist.angular.z

        self.odom_pub.publish(self.odom)

if __name__ == '__main__':
    rospy.init_node('Odometry_Conversion', anonymous=False)
    hz = rospy.get_param('/flyboi/sampling_frequency')
    r = rospy.Rate(hz)  #hz
    rospy.loginfo('Odometry Conversion Node Initialized')
    do_it = SubscribeAndPublishOdom()

    while not rospy.is_shutdown():
        rospy.spin()
