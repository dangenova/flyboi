#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

class ThustClassCrazyFlie():
    def __init__(self): 
        #get params
        self.publish_topic_name = rospy.get_param("~publish_topic_name", "cmd_vel" )
        self.publisher = rospy.Publisher(self.publish_topic_name, Twist, queue_size=1)
        self.publisher_stamp = rospy.Publisher('/crazyflie/cmd_vel_stamp', TwistStamped, queue_size=1)
        self.command_msg = Twist()
        self.command_msg_stamp = TwistStamped()


    def Read(self, roll,pitch,yawrate, thrust):
        self.command_msg.linear.y = roll
        self.command_msg.linear.x = pitch 
        self.command_msg.angular.z = yawrate
        self.command_msg.linear.z =  thrust

        self.command_msg_stamp.twist.linear = self.command_msg.linear
        self.command_msg_stamp.twist.angular = self.command_msg.angular
        
    
    def Publish(self):
        self.command_msg_stamp.header.stamp = rospy.Time.now()

        self.publisher.publish(self.command_msg)
        self.publisher_stamp.publish(self.command_msg_stamp)
