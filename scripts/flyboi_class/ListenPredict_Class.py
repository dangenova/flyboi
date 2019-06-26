#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
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
        self.x_tilde = np.zeros(self.hor_l)
        self.y_tilde = np.zeros(self.hor_l)
        self.z_tilde = np.zeros(self.hor_l)
        
        self.sub = rospy.Subscriber('/target/trajectory', PoseStamped, self.callback)
        self.OLS = ols.OLS(self.his_hz, self.fut_hz, self.hor_l, self.his_l, ols_factor)
        self.TrajectoryPublisher = traj.Trajectory(self.fut_hz, self.hor_l)

        self.flag = 1
    def callback(self, data):
        #shift data such that earliest position is at x[0]
        self.x_h = np.roll(self.x_h, -1)
        self.y_h = np.roll(self.y_h, -1)
        self.z_h = np.roll(self.z_h, -1)
        #subscribe to data
        self.x_h[self.his_l-1] = data.pose.position.x
        self.y_h[self.his_l-1] = data.pose.position.y
        self.z_h[self.his_l-1] = data.pose.position.z
        #update OLS and evoke OLS commands
        self.OLS.read(self.x_h, self.y_h, self.z_h)
        self.x_tilde, self.y_tilde, self.z_tilde = self.OLS.update()
        #Publsih the Trajectory
        
        self.TrajectoryPublisher.UpdateAndPublsih(self.x_tilde, self.y_tilde, self.z_tilde)
        if self.flag == 1:
            self.flag = 0
            rospy.loginfo('Trajectory is Publishing')
            
