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
    def __init__(self):
        self.hz = 50
        self.dt = 1.0/self.hz
        total_time = 30
        self.length = total_time*self.hz
        
        
        self.header = std_msgs.msg.Header()
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.velocities = Twist()
        self.accelerations = Twist()
        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory,queue_size=1)
        self.msg = MultiDOFJointTrajectory()
        self.msg.points = [0]*self.length
        self.msg.header = self.header

        self.speed_mod1 = .6
        self.speed_mod2 = .75
        self.speed_mod3 = .95
        self.mag = 1


        #bunch of stuff
        self.t_o = 0
        self.t_1 = 5+ self.t_o
        self.t_2 = self.t_1 + 2.0*math.pi/self.speed_mod1
        self.t_3 = self.t_2 + 2.0*math.pi/self.speed_mod2
        self.t_4 = self.t_3 + 2.0*math.pi/self.speed_mod3
        self.t_rest = self.t_4 +3.0
        self.t_5 = self.t_rest + 2.0*math.pi/self.speed_mod1
        self.t_6 = self.t_5 + 2.0*math.pi/self.speed_mod2
        self.t_7 = self.t_6 + 2.0*math.pi/self.speed_mod3

    def Update(self):
        self.msg.header.stamp = rospy.Time.now()
        for i in range(0,self.length):

            if i*self.dt <= self.t_1:
                x = 0
                y = 0
                z = 5
            
            elif i*self.dt >self.t_1 and i*self.dt <=self.t_2:
                x = self.mag * math.sin(
                    self.speed_mod1*(i*self.dt-self.t_1))
                y = 0
                z = 5
            
            elif i*self.dt>self.t_2 and i*self.dt <=self.t_3:
                x = self.mag * math.sin(
                    self.speed_mod2*(i*self.dt-self.t_2))
                y = 0
                z = 5
            
            elif i*self.dt>self.t_3 and i*self.dt <=self.t_4:
                x = self.mag * math.sin(
                    self.speed_mod3*(i*self.dt-self.t_3))
                y = 0
                z = 5
            
            elif i*self.dt >self.t_4 and i*self.dt <=self.t_5:
                x = 0
                y = 0
                z = 5
            
            else:
                x = 0
                y = 0
                z = 5
            
            transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
            point = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i*self.dt))
            self.msg.points[i] = point
            
        

    def Publish(self):
        self.traj_pub.publish(self.msg)
        rospy.loginfo('Should have published')


if __name__ == '__main__':
    rospy.init_node('Position_Reference_Node', anonymous=False)
    rospy.loginfo('Inititilize Reference Node')
    path = Trajectory()
    path.Update()
    path.Publish()
    
    while not rospy.is_shutdown():
        rospy.spin()
