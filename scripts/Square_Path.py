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

        

        self.first_run_flag = 0
        self.publish_oscilate_flag = 0
        self.finish_flag = 0
        self.rest_flag = 0


        # stuff for sys id path

        
        self.velocity_x = .4
        self.velocity_y = .4 
        self.distance = 1.5
        self.x_hold1 = 4
        self.y_hold1 = 4
        self.hz = 50
        self.dt = 1.0/self.hz
        

        x_o = -1.3
        y_o = -1.3
        z_o = 1

        self.t_o = 5
        t_1 = self.t_o + self.distance/self.velocity_x
        t_2 = t_1 + self.distance/self.velocity_y
        t_3 = t_2 + self.distance/self.velocity_x
        t_4 = t_3 + self.distance/self.velocity_y
        array_length_float = self.hz*(t_4)
        self.array_length = int(array_length_float)
        rospy.loginfo(self.array_length)
        self.x_array = np.zeros(self.array_length)
        self.y_array = np.zeros(self.array_length)
        self.z_array = np.zeros(self.array_length)
        self.w_array = np.zeros(self.array_length)

        for i in range(0,self.array_length):
            t = i*self.dt
            
            if t<= self.t_o:
                self.x_array[i] = x_o
                self.y_array[i] = y_o
                self.z_array[i] = z_o
                
                self.w_array[i] = 0
            
            elif t > self.t_o and t <= t_1:
                self.x_array[i] = x_o+self.velocity_x*(t-self.t_o)
                self.y_array[i] = y_o
                self.z_array[i] = z_o
                
                self.w_array[i] = 1
                x_hold = self.x_array[i]
            
            
            elif t > t_1 and t <= t_2:
                self.x_array[i]= x_hold
                self.y_array[i] = y_o+self.velocity_y*(t-t_1)
                self.z_array[i] = z_o

                self.w_array[i] = 1
                y_hold = self.y_array[i]

            elif t > t_2 and t <=t_3:
                self.x_array[i] = x_hold-self.velocity_x*(t-t_2)
                self.y_array[i] = y_hold
                self.z_array[i] = z_o

                self.w_array[i] = 1
                x_hold1 = self.x_array[i]

            elif t > t_3 and t <= t_4:
                self.x_array[i] = x_hold1
                self.y_array[i] = y_hold-self.velocity_y*(t-t_3)
                self.z_array[i] = z_o
                self.w_array[i] = 1

            else: 
                self.x_array[i] = x_o
                self.y_array[i] = y_o
                self.z_array[i] = z_o

                self.w_array[i] = 1


        # Variable to keep track  in callback
        self.array_position = 0
        rospy.Timer(rospy.Duration(.02), self.callback)

    def callback(self, data):
        self.pos_pub.publish(path.update())
        
    def update(self):

        self.msg.header.stamp = rospy.Time.now()
        self.msg.pose.position.x = self.x_array[self.array_position]
        self.msg.pose.position.y = self.y_array[self.array_position]
        self.msg.pose.position.z = self.z_array[self.array_position]
        self.msg.pose.orientation.w = self.w_array[self.array_position]
        
        self.array_position+= 1

        if self.array_position >= self.array_length:
            self.array_position = self.t_o*self.hz

        return self.msg

if __name__ == '__main__':
    rospy.init_node('Position_Reference_Node', anonymous=False)
    rospy.loginfo('Inititilize Reference Node')

    path = Path()

    while not rospy.is_shutdown():
        rospy.spin()