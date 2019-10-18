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

        self.ref_msg = PoseStamped()
        self.msg = MultiDOFJointTrajectory()
        self.header = std_msgs.msg.Header()

        self.future_length = 20
        self.dt = 0.1
        self.msg.points = [0]*self.future_length

        self.quaternion = tf.transformations.quaternion_from_euler(self.roll_o, self.pitch_o, self.yaw_o)
        self.ref_msg.pose.orientation.x = self.quaternion[0]
        self.ref_msg.pose.orientation.y = self.quaternion[1]
        self.ref_msg.pose.orientation.z = self.quaternion[2]
        self.ref_msg.pose.orientation.w = 0
        self.print_flag = 0

        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory,queue_size=10)
        self.ref_pub = rospy.Publisher('/target/position', PoseStamped ,queue_size=10)
        
        

        

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
        self.dt_array = 1.0/self.hz
        

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
        self.vx_array = np.zeros(self.array_length)
        self.vy_array = np.zeros(self.array_length)
        self.vz_array = np.zeros(self.array_length)
        self.w_array = np.zeros(self.array_length)

        for i in range(0,self.array_length):
            t = i*self.dt_array
            
            if t<= self.t_o:
                self.x_array[i] = x_o
                self.y_array[i] = y_o
                self.z_array[i] = z_o

                self.w_array[i] = 0

                self.vx_array[i] = 0.0
                self.vy_array[i] = 0.0
                self.vz_array[i] = 0.0
            
            elif t > self.t_o and t <= t_1:
                self.x_array[i] = x_o+self.velocity_x*(t-self.t_o)
                self.y_array[i] = y_o
                self.z_array[i] = z_o
                x_hold = self.x_array[i]

                self.w_array[i] = 1

                self.vx_array[i] = self.velocity_x
                self.vy_array[i] = 0.0
                self.vz_array[i] = 0.0
            
            
            elif t > t_1 and t <= t_2:
                self.x_array[i]= x_hold
                self.y_array[i] = y_o+self.velocity_y*(t-t_1)
                self.z_array[i] = z_o
                y_hold = self.y_array[i]

                self.w_array[i] = 1

                self.vx_array[i] = 0.0
                self.vy_array[i] = self.velocity_y
                self.vz_array[i] = 0.0

            elif t > t_2 and t <=t_3:
                self.x_array[i] = x_hold-self.velocity_x*(t-t_2)
                self.y_array[i] = y_hold
                self.z_array[i] = z_o
                x_hold1 = self.x_array[i]

                self.w_array[i] = 1

                self.vx_array[i] = -self.velocity_x
                self.vy_array[i] = 0.0
                self.vz_array[i] = 0.0

            elif t > t_3 and t <= t_4:
                self.x_array[i] = x_hold1
                self.y_array[i] = y_hold-self.velocity_y*(t-t_3)
                self.z_array[i] = z_o

                self.w_array[i] = 1

                self.vx_array[i] = 0.0
                self.vy_array[i] = -self.velocity_y
                self.vz_array[i] = 0.0

            else: 
                self.x_array[i] = x_o
                self.y_array[i] = y_o
                self.z_array[i] = z_o
                self.w_array[i] = 1


        # Variable to keep track  in callback
        self.array_position = 0

        rospy.Timer(rospy.Duration(.02), self.callback)

    def callback(self, data):
        self.ref_pub.publish(path.ref_update())
        self.traj_pub.publish(path.traj_update())

        self.array_position+= 1

        if self.array_position >= self.array_length:
            self.array_position = self.t_o*self.hz
        
    def ref_update(self):

        self.ref_msg.header.stamp = rospy.Time.now()
        self.ref_msg.pose.position.x = self.x_array[self.array_position]
        self.ref_msg.pose.position.y = self.y_array[self.array_position]
        self.ref_msg.pose.position.z = self.z_array[self.array_position]
        self.ref_msg.pose.orientation.w = self.w_array[self.array_position]

        return self.ref_msg
    
    def traj_update(self):
        self.msg.header.stamp = rospy.Time.now()
        for i in range(0,self.future_length):
            velocities = Twist()
            accelerations = Twist()

            # needed just in case the array bounds are overflowed
            j = self.array_position + i*5
            if j >= self.array_length:
                j = self.t_o*self.hz 

            x = self.x_array[j]
            y = self.y_array[j]
            z = self.z_array[j]

            velocities.linear.x = self.vx_array[j]
            velocities.linear.y = self.vy_array[j]
            velocities.linear.z = self.vz_array[j]
            accelerations.linear.x = 0.0
            accelerations.linear.y = 0.0
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