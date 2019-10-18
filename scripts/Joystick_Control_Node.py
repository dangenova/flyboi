#!/usr/bin/env python
# Joystick buttons mapping
# Repurposed Code from Peng Wei by Dan Genova
# Used to Determine UAV Mode

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class Controller():
	def __init__(self, joy_topic):
		rospy.wait_for_service('emergency')
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		#rospy.wait_for_service('land')
		self._land = rospy.ServiceProxy('landing_mode', Empty)
		#rospy.wait_for_service('takeoff')
		self._takeoff = rospy.ServiceProxy('takeoff_mode', Empty)

		self._mpc_control = rospy.ServiceProxy('mpc_control_mode', Empty)
		
		self._pid_control = rospy.ServiceProxy('pid_control_mode', Empty)

		# subscribe to the joystick at the end to make sure that all required
		# services were found
		self._buttons = None
		rospy.Subscriber(joy_topic, Joy, self._joyChanged)

	def _joyChanged(self, data):
		for i in range(0, len(data.buttons)):
			if self._buttons == None or data.buttons[i] != self._buttons[i]:
				if i == 0 and data.buttons[i] == 1:
					self._land()
				if i == 1 and data.buttons[i] == 1:
					self._pid_control()
				if i == 2 and data.buttons[i] == 1:
					self._takeoff()
					#rospy.loginfo("TakeOff requested!")
				#if i == 3 and data.buttons[i] == 1:
					#self._switch2consensus()
				#if i == 4 and data.buttons[i] == 1:
					#self._switch2standby()
				if i == 5 and data.buttons[i] == 1:
					self._mpc_control()
					#rospy.loginfo("Switch to line formation!")
				#if i == 6 and data.buttons[i] == 1:
					#rospy.loginfo("Update Params!")
		self._buttons = data.buttons

if __name__ == '__main__':
	rospy.init_node('Joystick_Control_Node', anonymous=True)
	joy_topic = rospy.get_param("~joy_topic", "/joy")
	controller = Controller(joy_topic)  
	rospy.spin()
