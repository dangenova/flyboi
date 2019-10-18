#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class ReferenceSubscribe():
    def __init__(self):
        self.topic_name = rospy.get_param("~uav_pose_reference_topic_name", "/command/pose")
        self.subscribe = rospy.Subscriber(self.topic_name, PoseStamped, self.callback)

    def callback(self,data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.z = data.pose.position.z

        self.qx = data.pose.orientation.x
        self.qy = data.pose.orientation.y
        self.qz = data.pose.orientation.z
        self.qw = data.pose.orientation.w

        quaternion = [self.qx, self.qy, self.qz, self.qw]
        euler = euler_from_quaternion(quaternion)
        
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

    def readPoseYaw(self):
        x = self.x
        y = self.y
        z = self.z
        yaw = self.yaw

        return x,y,z, yaw
        


