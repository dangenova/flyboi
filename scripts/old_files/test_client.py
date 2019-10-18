#!/usr/bin/env python
# Joystick buttons mapping
# Repurposed Code from Peng Wei

import rospy
from flyboi_ethz.srv import CrazyfliePID


class Client():
    def __init__(self):
        rospy.Timer(rospy.Duration(1), self.Timer_Callback)
        self.pid_calculation_client = rospy.ServiceProxy('PID_Calculation',CrazyfliePID)

    def Timer_Callback(self, event):
        yo=  self.pid_calculation_client(
                1,1,1,1,1,1,1, 1, 1, 1)
        
        rospy.loginfo('roll in  N %s'%yo.actuation_roll)
        rospy.loginfo('yawreate in  N %s'%yo.actuation_yawrate)


        




if __name__ == '__main__':
    rospy.init_node('Test_Client', anonymous=True)

    do_it = Client()

    while not rospy.is_shutdown():
        rospy.spin()
