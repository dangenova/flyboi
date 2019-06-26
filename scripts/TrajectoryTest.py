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
    def __init__(self, x_o, y_o, z_o, frequency, future_length):
        self.x_o = x_o
        self.y_o = y_o
        self.z_0 = z_o
        self.t = 0
        self.hz = frequency
        self.dt = 1.0/frequency
        self.length = future_length
        #self.msg = MultiDOFJointTrajectory()
        self.header = std_msgs.msg.Header()
        self.header.frame_id = 'frame'
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        
        self.msg = MultiDOFJointTrajectory()
        self.msg.points = [0]*self.length
        
        #transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
        self.velocities = Twist()
        self.accelerations = Twist()
       # p = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i*self.dt))

        self.count = 0
        #self.msg.points = 2
        #rospy.Time()
        #self.msg = MultiDOFJointTrajectory()
        #p = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i*self.dt))

    def update(self):
        #self.msg = MultiDOFJointTrajectory()
        rospy.loginfo(self.msg.points)
        for i in range(0,self.length):
            self.header.stamp = rospy.get_time()
            self.msg.header = self.header
            x = i+1
            y = i+2
            z = i+3
            transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
            p = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i*self.dt))
            rospy.loginfo(i)
            self.msg.points[i] = p
            #self.msg.points.append(p)
            #self.msg.points(i).position.y
            
            #self.msg.points(i).positions.z   
        #rospy.loginfo('Im printing nerd')
          
        rospy.loginfo('')
        rospy.loginfo(self.dt)
        rospy.loginfo('New Point %f', self.count)
        rospy.loginfo(self.msg)
        self.count =self.count+1
        return self.msg




if __name__ == '__main__':
    rospy.init_node('Trajectory_Test', anonymous=False)
    hz = .25
    r = rospy.Rate(hz)  #hz
    rospy.loginfo('ProgramSTarted')
    traj_pub = rospy.Publisher('/trajectory', MultiDOFJointTrajectory,queue_size=1)
    x_o = 1.0
    y_o = 1.0
    z_o = 5.0
    future_length = 4

    do_it = Trajectory(x_o, y_o, z_o, hz, future_length)

    while not rospy.is_shutdown():
        t = rospy.get_time()
        traj_pub.publish(do_it.update())
        r.sleep()