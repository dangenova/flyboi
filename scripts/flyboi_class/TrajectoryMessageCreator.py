#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import message_filters
import std_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Point, Twist

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class Trajectory():
    def __init__(self, frequency, future_length):
        self.hz = frequency
        self.dt = 1.0/frequency
        self.future_length = future_length
        self.header = std_msgs.msg.Header()
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.velocities = Twist()
        self.accelerations = Twist()
        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory,queue_size=10)
        self.msg = MultiDOFJointTrajectory()
        self.msg.points = [0]*self.future_length
        self.msg.header = self.header

    def UpdateAndPublish(self, x_tilde, y_tilde, z_tilde, vx_tilde, vy_tilde, vz_tilde, ax_tilde, ay_tilde, az_tilde):
        self.msg.header.stamp = rospy.Time.now()
        #to = rospy.Time.now()
        for i in range(0,self.future_length):
            velocities = Twist()
            accelerations = Twist()
            x = x_tilde[i]
            y = y_tilde[i]
            z = z_tilde[i]
            velocities.linear.x = vx_tilde[i]
            velocities.linear.y = vy_tilde[i]
            velocities.linear.z = vz_tilde[i]
            accelerations.linear.x = ax_tilde[i]
            accelerations.linear.y = ay_tilde[i]
            accelerations.linear.z = az_tilde[i]

            transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
            point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(i*self.dt))
            self.msg.points[i] = point

        self.traj_pub.publish(self.msg)