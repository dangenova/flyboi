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

class ListenToOdomCrazyFlie():
    def __init__(self):
        #Stuff for message sync
        self.position_sub = message_filters.Subscriber('/crazyflie/log1', GenericLogData)
        self.euler_angle_sub = message_filters.Subscriber('/crazyflie/log2', GenericLogData)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.position_sub, self.euler_angle_sub],2,.02)
        #self.ts = message_filters.TimeSynchronizer([self.position_sub, self.angle_sub],2)
        self.ts.registerCallback(self.callback)

        #Publish Odometry
        self.odom_pub = rospy.Publisher('/crazyflie/odom', Odometry, queue_size=10 )
        self.odom_mpc_pub = rospy.Publisher('/crazyflie/odom_mpc', Odometry, queue_size=10 )
        
        #Initilize Odometry Matrix
        self.odom = Odometry()
        self.cov_matrix_pub = np.zeros(36)
        cov_matrix = np.identity(6)*0.01 
        self.cov_matrix_pub = cov_matrix.flatten()
        self.odom.pose.covariance = self.cov_matrix_pub
        self.odom.twist.covariance = self.cov_matrix_pub

        self.odom_mpc = Odometry()
        self.cov_matrix_pub = np.zeros(36)
        self.odom_mpc.pose.covariance = self.cov_matrix_pub
        self.odom_mpc.twist.covariance = self.cov_matrix_pub


        #Message Header
        self.odom.header = std_msgs.msg.Header()

        rospy.loginfo('Crazyflie Odometry Initialized')

        #Flags
        self.first_callback_flag = 1

        #Stuff For reading Back
        self.roll = 0
        self.yaw = 0
        self.pitch = 0

        #conversion coeficients
        self.deg2rad = math.pi/180

        

    def callback(self, position, angle_e):

        if self.first_callback_flag == 1:
            rospy.loginfo('Crazyflie Wrapper Receives first UAV Odometry')
            self.first_callback_flag = 0


        self.roll = angle_e.values[0]*self.deg2rad
        self.pitch = angle_e.values[1]*self.deg2rad
        self.yaw = angle_e.values[2]*self.deg2rad

        q = quaternion_from_euler(self.roll ,self.pitch, self.yaw)
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


        # MPC Pitch is opposite of the Crazyflie
        q_mpc = quaternion_from_euler(self.roll ,-self.pitch, self.yaw)
        self.odom_mpc.pose.pose.position.x = position.values[0]
        self.odom_mpc.pose.pose.position.y = position.values[1]
        self.odom_mpc.pose.pose.position.z = position.values[2]
        self.odom_mpc.pose.pose.orientation.x = q_mpc[0]
        self.odom_mpc.pose.pose.orientation.y = q_mpc[1]
        self.odom_mpc.pose.pose.orientation.z = q_mpc[2]
        self.odom_mpc.pose.pose.orientation.w = q_mpc[3]
        

        v_x, v_y, v_z = self.WorldToBody(
            self.roll, -self.pitch, self.yaw, position.values[3], position.values[4], position.values[5] )

        self.odom_mpc.twist.twist.linear.x = v_x
        self.odom_mpc.twist.twist.linear.y = v_y
        self.odom_mpc.twist.twist.linear.z = v_z
        self.odom_mpc.twist.twist.angular.x = angle_e.values[3]
        self.odom_mpc.twist.twist.angular.y = -angle_e.values[4]
        self.odom_mpc.twist.twist.angular.z = angle_e.values[5]

        
    def Publish(self):
        self.odom.header.stamp = rospy.Time.now()
        self.odom_pub.publish(self.odom)

        self.odom_mpc.header.stamp = rospy.Time.now() 
        self.odom_mpc_pub.publish(self.odom_mpc)
    
    def ReadAll(self):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        z = self.odom.pose.pose.position.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw

        return x,y,z,roll,pitch,yaw

    def WorldToBody(self, phi, theta, psi, x, y, z):
        #input needs to be in deg

        sp = math.sin(phi)
        cp = math.cos(phi)
        st = math.sin(theta)
        ct = math.cos(theta)
        ss = math.sin(psi)
        cs = math.cos(psi)

        x_body = ct*cs*x + ct*ss*y -st*z
        y_body = (sp*st*cs-cp*ss)*x + (sp*st*ss+cp*cs)*y + sp*ct*z
        z_body = (cp*st*cs+sp*ss)*x + (cp*st*ss-sp*cs)*y+cp*ct*z

        return x_body, y_body, z_body


    
    def ReadPose(self):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        z = self.odom.pose.pose.position.z

        return x, y, z