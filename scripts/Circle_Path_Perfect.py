#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_msgs.msg
from nav_msgs.msg import Odometry

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint



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

        self.future_length = 20
        self.dt = 0.1
        
        
        self.ref_msg = PoseStamped()
        self.msg = MultiDOFJointTrajectory()
        self.header = std_msgs.msg.Header()
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory,queue_size=10)
        self.ref_pub = rospy.Publisher('/target/position', PoseStamped ,queue_size=10)
        self.msg.points = [0]*self.future_length


        self.print_flag = 0

        self.subscribe_topic_name= rospy.get_param('~SubscribeTopic', "/ardu/odom")
        self.subscribe = rospy.Subscriber(self.subscribe_topic_name, Odometry , self.callback)

        self.first_run_flag = 0
        self.publish_oscilate_flag = 0
        self.finish_flag = 0
        self.rest_flag = 0


        


        self.mag = .5
    
    def callback(self, data):
        if self.first_run_flag == 0:
            self.x_o = -.45
            self.y_o = -.45
            self.z_o = data.pose.pose.position.z
            self.t_o = rospy.get_time()
            self.first_run_flag +=1
        self.t = rospy.get_time()
        self.traj_pub.publish(path.traj_update())
        self.ref_pub.publish(path.ref_update())


    def ref_update(self):
        self.ref_msg.pose.position.x = self.x_o+self.mag*math.sin(self.t-self.t_o)
        self.ref_msg.pose.position.y = self.y_o+self.mag*math.cos(self.t-self.t_o)
        self.ref_msg.pose.position.z = self.z_o

        if self.t-self.t_o >5 :
            self.ref_msg.pose.orientation.w = 1

            
        self.ref_msg.header.stamp = rospy.Time.now()
        return self.ref_msg

    def traj_update(self):
        self.msg.header.stamp = rospy.Time.now()
        for i in range(0,self.future_length):
            velocities = Twist()
            accelerations = Twist()
            x = self.x_o+self.mag*math.sin((self.t-self.t_o)+i*self.dt)
            y = self.y_o+self.mag*math.cos((self.t-self.t_o)+i*self.dt)
            z = self.z_o
            velocities.linear.x = self.mag*math.cos((self.t-self.t_o)+i*self.dt)
            velocities.linear.y = -self.mag*math.sin((self.t-self.t_o)+i*self.dt)
            velocities.linear.z = 0.0
            accelerations.linear.x = -self.mag*math.sin((self.t-self.t_o)+i*self.dt)
            accelerations.linear.y = -self.mag*math.cos((self.t-self.t_o)+i*self.dt)
            accelerations.linear.z = 0.0

            transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
            point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(i*self.dt))
            self.msg.points[i] = point

        return self.msg

if __name__ == '__main__':
    rospy.init_node('Position_Reference_Node', anonymous=False)
    rospy.loginfo('Inititilize Reference Node')

    path = Path()

    while not rospy.is_shutdown():
        rospy.spin()