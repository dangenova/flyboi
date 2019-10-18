#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from flyboi_class import Ordinary_Least_Squares as ols
from flyboi_class import TrajectoryMessageCreator as traj

class ListenAndPredict():
    def __init__(self, history_frequency, future_frequency,  horizon, length_history, ols_factor):
        self.his_hz = history_frequency
        self.fut_hz = future_frequency
        self.hor_l = horizon
        self.his_l = length_history
        self.x_h = np.zeros(self.his_l)
        self.y_h = np.zeros(self.his_l)
        self.z_h = np.zeros(self.his_l)
        
        #add two in order to send full velocity/acceleration command
        self.x_tilde = np.zeros(self.hor_l)
        self.y_tilde = np.zeros(self.hor_l)
        self.z_tilde = np.zeros(self.hor_l)



        uav_sub_topic = rospy.get_param('~UAV_Odom_Topic')

        rospy.loginfo('History frequency is %s', history_frequency)
        rospy.loginfo('Future frequency is %s', future_frequency)
        rospy.loginfo('Horizon length is %s', horizon)
        rospy.loginfo('History length is %s', length_history)
        rospy.loginfo('OLS factor is %s', ols_factor)
        


        self.uav_sub = rospy.Subscriber(uav_sub_topic, Odometry, self.uav_callback)
        self.target_sub = rospy.Subscriber('/target/position', PoseStamped, self.target_callback)
        self.OLS = ols.OLS(self.his_hz, self.fut_hz, self.hor_l, self.his_l, ols_factor)
        self.TrajectoryPublisher = traj.Trajectory(self.fut_hz, self.hor_l)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.time_store = np.zeros(100)
        self.track = 0

    def target_callback(self, data):
        to = rospy.get_time()
        #shift data such that earliest position is at x[0]
        self.x_h = np.roll(self.x_h, -1)
        self.y_h = np.roll(self.y_h, -1)
        self.z_h = np.roll(self.z_h, -1)
        #subscribe to data
        self.x_h[self.his_l-1] = data.pose.position.x
        self.y_h[self.his_l-1] = data.pose.position.y
        self.z_h[self.his_l-1] = data.pose.position.z
        #update OLS and evoke OLS commands
        self.OLS.read(
            self.x_h, self.y_h, self.z_h, self.roll, self.pitch, self.yaw)
        self.x_tilde, self.y_tilde, self.z_tilde, self.vx_tilde, self.vy_tilde, self.vz_tilde, self.ax_tilde, self.ay_tilde, self.az_tilde  = self.OLS.update()
        # self.vx_tilde, self.vy_tilde, self.vz_tilde = self.OLS.getVeloctityHorizon()
        # self.ax_tilde, self.ay_tilde, self.az_tilde = self.OLS.getAccelerationHorizon()
        
        self.TrajectoryPublisher.UpdateAndPublish(
            self.x_tilde, self.y_tilde, self.z_tilde,
            self.vx_tilde, self.vy_tilde, self.vz_tilde,
            self.ax_tilde, self.ay_tilde, self.az_tilde)
        
        # rospy.loginfo('X - Tilde')
        # rospy.loginfo(self.x_h)
        # rospy.loginfo('current x')
        # rospy.loginfo(data.pose.position.x)
        #  JUST A BUNCH OF TIME KEEPING CRAP
        # t = rospy.get_time()
        # tdelta = t-to
        # rospy.loginfo('%s secs', tdelta)
        
        # self.time_store[self.track] = tdelta
        # self.track = self.track + 1
        
        # if self.track == 100:
        #     self.track = 0
        #     t_average = self.time_store.mean()
        #     rospy.loginfo('%s secs', t_average)
        #     rospy.loginfo('')
    
    def uav_callback(self, data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]


if __name__ == '__main__': 
    rospy.init_node('ListenAndPredict', anonymous=False)
    history_hz = rospy.get_param('/flyboi/sampling_frequency')
    future_hz = rospy.get_param('/flyboi/prediction_frequency')
    horizon = rospy.get_param('/flyboi/horizon_length') #number of points to calculate
    length_history = rospy.get_param('/flyboi/history_length_ols')  #number of points to calculate from
    ols_factor = rospy.get_param('/flyboi/ols_factor')
    r = rospy.Rate(history_hz)  #time in secs
    rospy.loginfo('Trajectory Prediction Initialized')
    
    do_it = ListenAndPredict(history_hz, future_hz, horizon, length_history, ols_factor)
    
    while not rospy.is_shutdown():
        rospy.spin()
