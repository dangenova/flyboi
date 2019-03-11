#include <cstdlib>
#include <math.h>
#include <std_msgs/String.h>

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>


geometry_msgs::PoseStamped loc_pos;
mavros_msgs::State UAV_state;
double local_roll;
double local_pitch;
double local_yaw;
float pi;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	loc_pos = *msg;
    tf::Quaternion q(
        loc_pos.pose.orientation.x,
        loc_pos.pose.orientation.y,
        loc_pos.pose.orientation.z,
        loc_pos.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(local_roll, local_pitch, local_yaw);
    
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    UAV_state = *msg;
   // mode_string = UAV_state.mode
}


int main(int argc, char **argv)
{
     pi = atan(1)*4;
    int rate = 4;
    int step;

    ros::init(argc, argv, "takeoff_initialize");
    ros::NodeHandle n;

    ros::Rate loop_rate(rate);

	ros::Subscriber loc_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_pos_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    /////////////////GUIDED//////////////////////
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
   // srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    
    ///////////////////ARM//////////////////////
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;

    /////////////////TAKEOFF////////////////////

    ros::ServiceClient takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    double height_limit = srv_takeoff.request.altitude - .1;

    ////////////////Adjust Attitude///////////////////
    tf2::Quaternion Q;
    ros::Publisher attitude_publisher;
    attitude_publisher = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    mavros_msgs::AttitudeTarget attitude_command;
    
    float roll, pitch, yaw;
    roll = pi*10.0/180.0;
    pitch = 0;
    yaw = 0;
    Q.setRPY(0,0,0);


    attitude_command.orientation.x = Q[0];
    attitude_command.orientation.y = Q[1];
    attitude_command.orientation.z = Q[2];
    attitude_command.orientation.w = Q[3];

    attitude_command.body_rate.x = 0;
    attitude_command.body_rate.y = 0;
    attitude_command.body_rate.z = 0;
        
    attitude_command.thrust = .5;
   

    /////////////// ADD SOMETHING HERE SAYING THAT THE PROGRAM IS GOING TO CLOSE BECAUSE THE UAV is already doing its thingc
    
    int count = 5;
    ROS_INFO("Checking current UAV STATE");
    while(count > 0 ) {
        count--;
        ros::spinOnce();
        loop_rate.sleep();
    }

    if(UAV_state.mode == "GUIDED" && !UAV_state.armed) {
        
        ROS_INFO("UAV already in GUIDED mode, will ARM soon");
        step = 2;
    }
    else if(UAV_state.mode == "GUIDED"  && UAV_state.armed && (loc_pos.pose.position.z < 1)) {
        ROS_INFO("UAV already in GUIDED mode and ARMED, will TAKEOFF Soon");
        step = 3;
    }
    else if(loc_pos.pose.position.z > 1.0) {
        ROS_INFO("Program Stopped Because UAV is already in Flight");
        return -1;
    }
    else {
        step = 1;
    }
    ////////////////ROS LOOP Setup/////////////
    ros::Time last_request = ros::Time::now();

    while (ros::ok()){

        switch(step) {
            
            case 1:
                if((ros::Time::now()-last_request > ros::Duration(3.0)) ) {
                    last_request = ros::Time::now();
                    if(set_mode_client.call(srv_setMode)  && srv_setMode.response.mode_sent) {
                        ROS_INFO("UAV IN GUIDED MODE");
                        if (UAV_state.armed) {
                            ROS_INFO("UAV already ARMED, Will TAKEOFF");
                            step = 3;
                        }

                        step++;
                    }
                }
                break;

            case 2:
                if((ros::Time::now()-last_request > ros::Duration(3.0)) ) {
                    last_request = ros::Time::now();
                    
                    if(arming_client.call(srv_arm)  && srv_arm.response.success) {
                        ROS_INFO("UAV ARMED");
                        step++;
                    }
                    else {
                        ROS_INFO("ARM FAILURE, will try again");
                    }
                }
                break;

            case 3:
                if((ros::Time::now()-last_request > ros::Duration(3.0)) ) {
                    last_request = ros::Time::now();
                    
                    if(takeoff_client.call(srv_takeoff) && srv_takeoff.response.success) {
                        ROS_INFO("Starting Takeoff");
                        step++;
                    }
                }
                break;

            case 4:
                if(loc_pos.pose.position.z > height_limit) {
                    ROS_INFO ("Takeoff Finished");
                    step++;
                }
                break;
            case 5:
                attitude_publisher.publish(attitude_command);
                if((local_yaw*180/pi) < 1 || (local_yaw*180/pi)>359) {
                    step++;
                    ROS_INFO("Takeoff and Orientation Finished, feel free to Exit Program");
                }
                ROS_INFO("Command Values: %f %f %f %f", attitude_command.orientation.x, attitude_command.orientation.y, attitude_command.orientation.z,attitude_command.orientation.w);
                ROS_INFO("Degrees Roll is %f, Pitch is %f, Yaw is %f", local_roll*180/pi,local_pitch*180/pi,local_yaw*180/pi);   
                ROS_INFO("Rads Roll is %f, Pitch is %f, Yaw is %f", local_roll,local_pitch,local_yaw);              
                break;
            case 6:
                break;          

        }

        //ROS_INFO("Roll is %f, Pitch is %f, Yaw is %f", local_roll*180/pi,local_pitch*180/pi,local_yaw*180/pi);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
