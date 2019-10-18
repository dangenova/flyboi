#!/usr/bin/env python
# Joystick buttons mapping
# Repurposed Code from Peng Wei

import rospy
from flyboi_ethz.srv import *
import math


class Server():
    def __init__(self):
        self.pid_calculation_server = rospy.Service('PID_Calculation',CrazyfliePID, self.callback)

    def callback(self, req):
        return CrazyfliePIDResponse(2,3,5,6)


        




if __name__ == '__main__':
    rospy.init_node('Test_Server', anonymous=True)

    rospy.loginfo(math.pi)
    do_it = Server()

    while not rospy.is_shutdown():
        rospy.spin()
