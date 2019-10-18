#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from enum import Enum
from geometry_msgs.msg import Twist

class Mission_Status(Enum):
    takeoff = 1
    control = 2

class Mission():
    def __init__(self):
        #get stuff from params
        self.status = Mission_Status.takeoff
        self.x = rospy.get_param("~x")
        self.y = rospy.get_param("~y")
        self.z = rospy.get_param("~z")
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.pose_topic_name = rospy.get_param("~pose_topic_name")

        #Pose Message for Takeoff
        self.pose_msg = PoseStamped()
        self.pose_msg.header.seq = 0
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_msg.header.frame_id = self.worldFrame
        self.pose_msg.pose.position.x = self.x
        self.pose_msg.pose.position.y = self.y
        self.pose_msg.pose.position.z = self.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.pose_msg.pose.orientation.x = quaternion[0]
        self.pose_msg.pose.orientation.y = quaternion[1]
        self.pose_msg.pose.orientation.z = quaternion[2]
        self.pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub = rospy.Publisher(self.pose_topic_name, PoseStamped, queue_size=1)

        #Pose Message for Takeoff
        self.command_msg = Twist()
        self.command_msg.header.seq = 0
        self.command_msg.header.stamp = rospy.Time.now()
        self.command_msg.header.frame_id = self.worldFrame

        self.command_pub = rospy.Publisher('cmd_vel', Twist)

        # Position Subscriber
        self.pose_sub = rospy.Subscriber('/')

    def update(self):
        if self.status = 

        


if __name__ == '__main__':
    rospy.init_node('Crazyflie_Test', anonymous=True)
    r = rospy.get_param("~rate")
    rate = rospy.Rate(r)



    pub = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
