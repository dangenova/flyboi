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


geometry_msgs::PoseStamped loc_pos;
mavros_msgs::State UAV_state;
//char mode_string[10];

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	loc_pos = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    UAV_state = *msg;
   // mode_string = UAV_state.mode
}


int main(int argc, char **argv)
{

    int rate = 10;

    ros::init(argc, argv, "takeoff_initialize");
    ros::NodeHandle n;

    ros::Rate r(rate);

	ros::Subscriber loc_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_pos_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher  loc_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    

    if(cl.call(srv_setMode)){
    	//ros::spinOnce();
        //ROS_INFO("%s",UAV_state.mode.c_str());
        /*if(UAV_state.mode == "GUIDED") {
            ROS_INFO("Entering GUIDED MODE");           
        }
        else {
            ROS_INFO("Failed to SetMode after service call");
            ROS_INFO("Exiting Program");
            return -1;            
        }
        */
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        //sleep(5);
        //ros::spinOnce();
        

        /*
        if(UAV_state.armed) {
            ROS_INFO("Armed");
            // ROS_ERROR("ARM send ok %d", srv.response.success);
        }
        else {
            ROS_INFO("Failure to Arm after service call");
            ROS_INFO("Exiting Program");
            return -1;
        }
        */
    }else{
        ROS_ERROR("Failed arming");
        return -1;
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////

    ROS_INFO("Trying to takeoff");

    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    


    if(takeoff_cl.call(srv_takeoff)){
    	ROS_INFO("Takeoff");
      //
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
        return -1;
    }

    
    // don't do anything while taking off
    double height_limit;
    height_limit = srv_takeoff.request.altitude - .1;
    ROS_INFO("Height Limit Set to %f", height_limit);
    
    while(loc_pos.pose.position.z < height_limit){
    	ros::spinOnce();
    } 

    ROS_INFO("Takeoff Complete");



/*
    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }
*/

    return 0;
}
