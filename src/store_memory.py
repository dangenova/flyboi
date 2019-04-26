#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from flyboi_ethz import Ordinary_Least_Squares as ols


class ListenAndPredict():
    def __init__(self, frequency, horizon, length_history):
        self.hz = frequency
        self.hor_l = horizon
        self.his_l = length_history
        self.x_h = np.zeros(self.his_l)
        self.y_h = np.zeros(self.his_l)
        self.x_tilde = np.zeros(self.hor_l)
        self.y_tilde = np.zeros(self.hor_l)
        self.sub = rospy.Subscriber('/command/pose', PoseStamped, self.callback)
        self.OLS = ols.OLS(self.hz, self.hor_l, self.his_l, 3)

    def callback(self, data):
        #shift data such that earliest position is at x[0]
        self.x_h = np.roll(self.x_h, -1)
        self.y_h = np.roll(self.y_h, -1)
        #subscribe to data
        self.x_h[self.his_l-1] = data.pose.position.x
        self.y_h[self.his_l-1] = data.pose.position.y
        #update OLS and evoke OLS commands
        self.OLS.read(self.x_h, self.y_h)
        self.x_tilde, self.y_tilde = self.OLS.update()

        rospy.loginfo('x matrix is %s',self.y_tilde)

if __name__ == '__main__': 
	rospy.init_node('ListenAndPredict', anonymous=False)
	hz = 10
	horizon = 10 #number of points to calculate
	length_history = 4 #number of points to calculate from
	r = rospy.Rate(10)  #time in secs
	rospy.loginfo('I exist')

	do_it = ListenAndPredict(hz, horizon, length_history)

	while not rospy.is_shutdown():
		rospy.spin()