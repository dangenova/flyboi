/*
  This code is to convert the ETHZ MPC Commands into a readible format by Mavros
*/



#include <cstdlib>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

//Define Pi as a Global Variable

float pi;

class SubscribePublishMPCCommand {
	private:
		mav_msgs::RollPitchYawrateThrust mpc_command;
		mavros_msgs::AttitudeTarget mavros_command;
		ros::NodeHandle n;
		ros::Subscriber mpc_sub;
		ros::Publisher mavros_pub;
		tf2::Quaternion Q;
		float max_thrust = 43.275; //max thrust for Mavros machine
		float mpc_thrust;
		int size;

	public: 
		
		SubscribePublishMPCCommand() {
			mpc_sub = n.subscribe("/command/roll_pitch_yawrate_thrust", 5, &SubscribePublishMPCCommand::callback, this);
			mavros_pub = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
		}

		void callback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg) {
			mpc_command = *msg;
			Q.setRPY(mpc_command.roll, mpc_command.pitch, 0);
			
			mavros_command.orientation.x = Q[0];
			mavros_command.orientation.y = Q[1];
			mavros_command.orientation.z = Q[2];
			mavros_command.orientation.w = Q[3];
			
			mavros_command.body_rate.x = 0;
			mavros_command.body_rate.y = 0;
			mavros_command.body_rate.z = mpc_command.yaw_rate;
		
			mavros_command.thrust = .5;

			//orient_pub.publish(angle_command);
			mavros_pub.publish(mavros_command);

			//ROS_INFO("Quaternion %f %f %f %f ", Q[0], Q[1], Q[2], Q[3]);
			//ROS_INFO("I CAN TALK");
			ROS_INFO("The thrust is is %f", mavros_command.thrust);
		}

		void print() {
			ROS_INFO("Print Works");
		}
		

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "MPC_Command_Convert");

  ROS_INFO("Before Class Declaration");

  SubscribePublishMPCCommand SAP_Obj;

  pi = atan(1)*4;
  int rate = 50;
  ros::Rate r(rate);
	
	//SAP_Obj.print();



  while (ros::ok()){
      ros::spinOnce();
      r.sleep();
  }    

  ROS_INFO("Not Spinning");
    
  return 0;
}

