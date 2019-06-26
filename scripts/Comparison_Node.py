#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import message_filters
import std_msgs.msg

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from flyboi_ethz.msg import pose_comparison

class PublishComparison():
    def __init__(self):
        #Stuff for message sync
        self.uav_sub = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        self.ref_sub = message_filters.Subscriber('/target/position', PoseStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.uav_sub, self.ref_sub], 2, .01)
        self.ts.registerCallback(self.callback)
        
        #Publish Odometry
        self.comparison_pub = rospy.Publisher('/compare_position',pose_comparison, queue_size=10 )
        
        #Initilize Msg
        self.msg = pose_comparison()
        self.header = std_msgs.msg.Header()
        self.msg.header = self.header
        self.flag = 1

        rospy.loginfo('Comparison Node Initialized')

    def callback(self, uav, ref):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.system.position.x = uav.pose.position.x
        self.msg.system.position.y = uav.pose.position.y
        self.msg.system.position.z = uav.pose.position.z
        self.msg.system.orientation.x = uav.pose.orientation.x
        self.msg.system.orientation.y = uav.pose.orientation.y
        self.msg.system.orientation.z = uav.pose.orientation.z
        self.msg.system.orientation.w = uav.pose.orientation.w

        self.msg.reference.position.x = ref.pose.position.x
        self.msg.reference.position.y = ref.pose.position.y
        self.msg.reference.position.z = ref.pose.position.z
        self.msg.reference.orientation.x = ref.pose.orientation.x
        self.msg.reference.orientation.y = ref.pose.orientation.y
        self.msg.reference.orientation.z = ref.pose.orientation.z
        self.msg.reference.orientation.w = ref.pose.orientation.w

        self.comparison_pub.publish(self.msg)
        if self.flag == 1:
            rospy.loginfo('Publishing Comparison for Bag File')
            self.flag =0
        
if __name__ == '__main__':
    rospy.init_node('Comparison_Node', anonymous=False)
    hz = rospy.get_param('/flyboi/sampling_frequency')
    r = rospy.Rate(hz)  #hz
    do_it = PublishComparison()

    while not rospy.is_shutdown():
        rospy.spin()
