#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_msgs.msg
from nav_msgs.msg import Odometry



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

        self.msg = PoseStamped()
        self.header = std_msgs.msg.Header()

        self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
        self.msg.pose.orientation.x = self.quaternion[0]
        self.msg.pose.orientation.y = self.quaternion[1]
        self.msg.pose.orientation.z = self.quaternion[2]
        self.msg.pose.orientation.w = 0
        self.print_flag = 0

        self.publish_topic_name= rospy.get_param('~PublishTopic', "/command/pose")
        self.pos_pub = rospy.Publisher(self.publish_topic_name, PoseStamped ,queue_size=10)

        rospy.Timer(rospy.Duration(.02), self.callback)

        self.first_run_flag = 0
        self.publish_oscilate_flag = 0
        self.finish_flag = 0
        self.rest_flag = 0


        # stuff for sys id path


        self.mag = .5
    
    def callback(self, data):
        if self.first_run_flag == 0:
            #set both to -.45 for physical test 
            self.x_o = -.45 #data.pose.pose.position.x
            self.y_o = -.45 #data.pose.pose.position.y
            self.z_o = 1.0
            self.t_o = rospy.get_time()
            self.first_run_flag +=1
        self.t = rospy.get_time()
        self.pos_pub.publish(path.update())

        


    def update(self):
        self.msg.pose.position.x = self.x_o+self.mag*math.sin(1.2*(self.t-self.t_o))
        self.msg.pose.position.y = self.y_o+self.mag*math.cos(1.2*(self.t-self.t_o))
        self.msg.pose.position.z = self.z_o

        if self.t-self.t_o >5 :
            self.msg.pose.orientation.w = 1
            
        self.msg.header.stamp = rospy.Time.now()
        return self.msg

if __name__ == '__main__':
    rospy.init_node('Position_Reference_Node', anonymous=False)
    rospy.loginfo('Inititilize Reference Node')

    path = Path()

    while not rospy.is_shutdown():
        rospy.spin()