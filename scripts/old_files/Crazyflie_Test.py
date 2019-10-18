#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from mav_msgs.msg import RollPitchYawrateThrust

class SubscribePublishMPCCommandCrazyflie():
    def __init__(self):
        #get params
        self.subscribe_topic_name= rospy.get_param("subscribe_topic_name", "/command/roll_pitch_yawrate_thrust")
        self.publish_topic_name = rospy.get_param("~publish_topic_name", "cmd_vel" )
        self.ThrustRate = rospy.get_param("~ThrustRateConstant", .000943) 
        self.ThrustOffset = rospy.get_param("~ThrustOffsetConstant", -5.537574)
        self.Newton2GramForceConstant = rospy.get_param("~Newton2GramForceConstant", .0098066)
        
        #setup subscriber and publisher
        self.subscribe = rospy.Subscriber(self.subscribe_topic_name, RollPitchYawrateThrust, self.callback)
        self.publisher = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=1)
        
        # Initialze Message
        self.command_msg = Twist()


        self.counter = 0
        self.overflow_counter = 0
    def callback(self,data):
        # Crazyflie driver will not work unless there is an initial thrust value of zero
        if self.counter < 10:
            self.command_msg.linear.y = 0
            self.command_msg.linear.x = 0
            self.command_msg.angular.z = 0
            self.command_msg.linear.z =  0
        else:
            #Basic Conversion
            self.command_msg.linear.y = data.roll
            self.command_msg.linear.x = data.pitch 
            self.command_msg.angular.z = data.yaw_rate
            self.command_msg.linear.z =  self.ThrustToPWMConversion(data.thrust.z)
        self.counter +=1
        self.publisher.publish(self.command_msg)

    def ThrustToPWMConversion(self, thrust_force_newton):
        thrust_force_gram = thrust_force_newton/self.Newton2GramForceConstant
        
        #Given T = A+B*PWM..... PWM = (T-A)/B
        PWM = (thrust_force_gram-self.ThrustOffset)/self.ThrustRate
        
        if self.overflow_counter == 0:
            #rospy.loginfo(PWM)
            rospy.loginfo('Thrust in gf %s' %thrust_force_gram)
            #rospy.loginfo('Thrust in  N %s'%thrust_force_newton)
            self.overflow_counter = 50
        self.overflow_counter -= 1

        return PWM

if __name__ == '__main__':
    rospy.init_node('Crazyflie_MPC_Command_Conversion', anonymous=True)
    hz = rospy.get_param("~rate", 50)
    rate = rospy.Rate(hz)  #hz
    rospy.loginfo('MPC Command Conversion for Crazyflie Initialized')

    do_it = SubscribePublishMPCCommandCrazyflie()
        
    while not rospy.is_shutdown():
        rospy.spin()
