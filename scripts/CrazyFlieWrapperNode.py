#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from mav_msgs.msg import RollPitchYawrateThrust
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from flyboi_ethz.srv import CrazyfliePID
from flyboi_class.CrazyFlieOdomClass import ListenToOdomCrazyFlie as Crazy_Odom_Sub
from flyboi_class.CrazyFlieSubscribeThrustClass import SubscribeThrustClassCrazyFlie as Crazy_MPC_Command_Sub
from flyboi_class.CrazyFlieThrustClass import ThustClassCrazyFlie as Generic_Thrust_Class
from flyboi_class.ReferenceSubscribeClass import ReferenceSubscribe as Reference_Sub
from flyboi_class.Joystick_Control_Class import Joystick as Joy_Control



class Wrapper():
    def __init__(self):
        self.hz = rospy.get_param("~rate", 50)
        self.takeoff_height = rospy.get_param("~takeoff_height", 1)
        self.dt = 1.0/self.hz
        self.MPCThrustCommand = Crazy_MPC_Command_Sub()
        self.Odometry = Crazy_Odom_Sub()
        self.ThrustCommand = Generic_Thrust_Class()
        self.Reference = Reference_Sub()
        
        self.Joystick_Command = Joy_Control()
        
        self.state = 'Idle'

        self.takeoff_server = rospy.Service('takeoff_mode', Empty, self.Takeoff_Service)
        self.land_server = rospy.Service('landing_mode', Empty, self.Landing_Service)
        self.mpc_control_server = rospy.Service('mpc_control_mode', Empty, self.MPC_Service)
        self.pid_control_server = rospy.Service('pid_control_mode', Empty, self.PID_Service)

        self.pid_calculation_client = rospy.ServiceProxy('PID_Calculation',CrazyfliePID)


        rospy.Timer(rospy.Duration(self.dt), self.Timer_Callback)

        self.start_z = 0
        self.counter = 0
        self.reset_PID_I = 0

        self.t_o = 0
    
    def Takeoff_Service(self, req):
        self.state = 'Takeoff'
        self.start_x, self.start_y, self.start_z = self.Odometry.ReadPose() 
        
        if self.start_z > .5:
            self.start_z = .05

        rospy.loginfo('TAKEOFF MODE')
        self.reset_PID_I = 1
        return EmptyResponse()
   
    def Landing_Service(self, req):
        self.state = 'Land'
        self.end_x, self.end_y, self.land_goalz = self.Odometry.ReadPose()
        self.previous_time = rospy.get_time()
        rospy.loginfo('LAND MODE')
        return EmptyResponse()
   
    def MPC_Service(self, req):
        self.state = 'MPC'
        rospy.loginfo('MPC CONTROL MODE')
        return EmptyResponse()
   
    def PID_Service(self, req):
        self.state = 'PID'
        rospy.loginfo('PID CONTROL MODE')
        return EmptyResponse()

    def Timer_Callback(self, event):
        self.counter += 1
        if self.counter == 100:
            #x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            
            #rospy.loginfo('x = %s y = %s z = %s', x, y, z)
            #rospy.loginfo('roll = %s pitch = %s yaw = %s', roll, pitch, yaw)
            #rospy.loginfo(' ')
            self.counter = 0
            
        
        if self.state == 'Idle':
            self.ThrustCommand.Read(0,0,0,0)
            self.ThrustCommand.Publish()
            self.Odometry.Publish()
        
        elif self.state == 'Takeoff':
                
            takeoff_goalz = self.start_z+self.takeoff_height

            x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            
            if self.reset_PID_I == 1:
                command = self.pid_calculation_client(
                    x,y,z,roll,pitch,yaw, self.start_x, self.start_y, takeoff_goalz, 0, True)
                self.reset_PID_I = 0
            
            command = self.pid_calculation_client(
                x,y,z,roll,pitch,yaw, self.start_x, self.start_y, takeoff_goalz, 0, False)
            
            self.ThrustCommand.Read(command.actuation_roll, command.actuation_pitch, command.actuation_yawrate, command.actuation_thrust_pwm)
            self.ThrustCommand.Publish()
            self.Odometry.Publish()
        
        elif self.state == 'Land':

            t = rospy.get_time()
            dt = t - self.previous_time

            x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            

            if z > self.start_z+.1:
                command  = self.pid_calculation_client(
                x,y,z,roll,pitch,yaw, self.end_x, self.end_y, self.land_goalz, 0, False)
            
                self.ThrustCommand.Read(command.actuation_roll, command.actuation_pitch, command.actuation_yawrate, command.actuation_thrust_pwm)
                self.ThrustCommand.Publish()
            else:
                self.state = 'Idle'
                rospy.loginfo('Finished Landing')
                rospy.loginfo('IDle Mode') 
            self.land_goalz = self.land_goalz - 0.15*dt 
            
            self.Odometry.Publish()

        elif self.state == 'MPC':

            #PID for Height because I'm a coward
            # x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            # x_ref, y_ref, z_ref, yaw_ref= self.Reference.readPoseYaw()

            # command = self.pid_calculation_client(
            #     x,y,z,roll,pitch,yaw, x_ref, y_ref, z_ref, yaw_ref, False)

            mpc_roll, mpc_pitch, mpc_yawrate, mpc_thrust  = self.MPCThrustCommand.Read()
            
            self.ThrustCommand.Read(mpc_roll, mpc_pitch, -mpc_yawrate, mpc_thrust)
            self.ThrustCommand.Publish()
            self.Odometry.Publish()
            


            # CODE FOR SYSTEM ID STUFF
            # x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            # x_ref, y_ref,roll_ref,pitch_ref, z_ref, yaw_ref = self.Odometry.ReadAll()
            # command = self.pid_calculation_client(
            #     x,y,z,roll,pitch,yaw, x, y, 1, 0, False)

            # roll_command = self.Joystick_Command.twist.linear.y
            # pitch_command = self.Joystick_Command.twist.linear.x
            # yaw_command = self.Joystick_Command.twist.angular.z
            
            # self.ThrustCommand.Read(roll_command, pitch_command, yaw_command, command.actuation_thrust_pwm)
            # self.ThrustCommand.Publish()
            # self.Odometry.Publish()

        elif self.state == 'PID':

            x,y,z,roll,pitch,yaw = self.Odometry.ReadAll()
            x_ref, y_ref, z_ref, yaw_ref= self.Reference.readPoseYaw()
            
            command = self.pid_calculation_client(
                x,y,z,roll,pitch,yaw, x_ref, y_ref, z_ref, yaw_ref, False)
            
            self.ThrustCommand.Read(command.actuation_roll, command.actuation_pitch, command.actuation_yawrate, command.actuation_thrust_pwm)
            self.ThrustCommand.Publish()
            self.Odometry.Publish()

        else:
            rospy.loginfo('UNSUPORTED MODE')
        
        self.previous_time = rospy.get_time()
        #self.MPCThrustCommand.Publish()
        #self.Odometry.Publish()



if __name__ == '__main__':
    rospy.init_node('Crazyflie_MPC_Wrapper', anonymous=True)

    do_it = Wrapper()

    while not rospy.is_shutdown():
        rospy.spin()

