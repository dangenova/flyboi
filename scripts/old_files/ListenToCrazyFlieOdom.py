#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import message_filters
from crazyflie_driver.msg import GenericLogData
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import std_msgs.msg

class ListenToCrazyFlieOdom():
    def __init__(self):
        #Stuff for message sync
        self.position_sub = message_filters.Subscriber('/crazyflie/log1', GenericLogData)
        self.euler_angle_sub = message_filters.Subscriber('/crazyflie/log2', GenericLogData)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.position_sub, self.euler_angle_sub],2,.02)
        #self.ts = message_filters.TimeSynchronizer([self.position_sub, self.angle_sub],2)
        self.ts.registerCallback(self.callback)

        #Publish Odometry
        self.odom_pub = rospy.Publisher('/crazyflie/odom', Odometry, queue_size=10 )
        
        #Initilize Odometry Matrix
        self.odom = Odometry()
        self.cov_matrix_pub = np.zeros(36)
        cov_matrix = np.identity(6)*.01 
        self.cov_matrix_pub = cov_matrix.flatten()
        self.odom.pose.covariance = self.cov_matrix_pub
        self.odom.twist.covariance = self.cov_matrix_pub

        #Message Header
        self.odom.header = std_msgs.msg.Header()

        

    def callback(self, position, angle_e):
        q = quaternion_from_euler(angle_e.values[0],angle_e.values[1], angle_e.values[2])
        self.odom.pose.pose.position.x = position.values[0]
        self.odom.pose.pose.position.y = position.values[1]
        self.odom.pose.pose.position.z = position.values[2]
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        

        self.odom.twist.twist.linear.x = position.values[3]
        self.odom.twist.twist.linear.y = position.values[4]
        self.odom.twist.twist.linear.z = position.values[5]
        self.odom.twist.twist.angular.x = angle_e.values[3]
        self.odom.twist.twist.angular.y = angle_e.values[4]
        self.odom.twist.twist.angular.z = angle_e.values[5]

        self.odom.header.stamp = rospy.Time.now()

        self.odom_pub.publish(self.odom)
if __name__ == '__main__':
    rospy.init_node('Crazyflie_Odometry_Conversion', anonymous=False)
    hz = rospy.get_param('~rate')
    r = rospy.Rate(hz)  #hz
    rospy.loginfo('Odometry Combination Node Starting')
    do_it = ListenToCrazyFlieOdom()

    while not rospy.is_shutdown():
        rospy.spin()
