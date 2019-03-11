#include <cstdlib>
#include <math.h>
#include <ros/ros.h>

#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/AttitudeTarget.h>


class SubscribePublishMPCCommand {
	private:
		geometry_msgs::PoseStamped angle_command; 
		geometry_msgs::PoseStamped info; 
		
		//mavros_msgs::Thrust thrust_command;
		mavros_msgs::AttitudeTarget thrust_command;
		ros::NodeHandle n;
		ros::Subscriber cmd_sub;
		ros::Publisher orient_pub;
		ros::Publisher thrust_pub;
		tf2::Quaternion Q;
		float max_thrust = 43.275; //max thrust for Mavros machine
		float mpc_thrust;
		int size;
		float pi = atan(1)*4;

		float roll = 0; 
		float pitch = 0;
		float yaw = 0;

	public: 
		
		SubscribePublishMPCCommand() {
			cmd_sub = n.subscribe("/firefly/command/pose", 1, &SubscribePublishMPCCommand::callback, this);
			//orient_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
			
			//thrust_pub = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 10);
    		thrust_pub = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    		
		}

		void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
			info = *msg;
			
			ROS_INFO("The X position is %f", msg->pose.position.x);

			Q.setRPY(roll, pitch, yaw);


			thrust_command.thrust = .5;
			thrust_command.orientation.x = Q[0];
			thrust_command.orientation.y = Q[1];
			thrust_command.orientation.z = Q[2];
			thrust_command.orientation.w = Q[3];
			thrust_command.body_rate.x = 0;
			thrust_command.body_rate.y = 0;
			thrust_command.body_rate.z = 0;


			//orient_pub.publish(angle_command);
			thrust_pub.publish(thrust_command);

			//ROS_INFO("Quaternion %f %f %f %f ", Q[0], Q[1], Q[2], Q[3]);
			//ROS_INFO("I CAN TALK");
			ROS_INFO("The thrust is is %f", thrust_command.thrust);
			ROS_INFO("Roll is %f Pitch is %f", roll, pitch);
		}

		void print() {
			ROS_INFO("Print Works");
		}
		

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "MPC_Command_Convert");

    ROS_INFO("Before Class Declaration");

    SubscribePublishMPCCommand SAP_Obj;
	
	int rate = 50;
    ros::Rate r(rate);
	
	//SAP_Obj.print();

	ROS_INFO("Edit 7");

    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }    

    ROS_INFO("Not Spinning");
    
    return 0;
}

