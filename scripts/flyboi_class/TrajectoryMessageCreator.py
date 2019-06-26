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
        self.header.frame_id = 'map'
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.velocities = Twist()
        self.accelerations = Twist()
        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory,queue_size=1)
        self.msg = MultiDOFJointTrajectory()
        self.msg.points = [0]*self.future_length
        self.msg.header = self.header

    def UpdateAndPublsih(self, x_tilde, y_tilde, z_tilde):
        self.msg.header.stamp = rospy.Time.now()
        for i in range(0,self.future_length):
            x = x_tilde[i]
            y = y_tilde[i]
            z = z_tilde[i]
            transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
            point = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i*self.dt))
            self.msg.points[i] = point
        #rospy.loginfo(self.msg)
        self.traj_pub.publish(self.msg)