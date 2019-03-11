#include <cstdlib>
#include <math.h>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>


geometry_msgs::PoseStamped loc_pos;
mavros_msgs::State UAV_state;


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	loc_pos = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    UAV_state = *msg;
}


int main(int argc, char **argv)
{

    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff_erle");
    ros::NodeHandle n;

    ros::Rate r(rate);

	ros::Subscriber loc_pos_sub = n.subscribe("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_pos_sub = n.subscribe("/mavros/state", 10, state_cb);
    ros::Publisher  loc_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    
    if(cl.call(srv_setMode)){
        ROS_INFO("Entering GUIDED MODE");
        //ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
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
        ros::spinOnce();
        if(UAV_state.armed) {
            ROS_INFO("Armed");
            // ROS_ERROR("ARM send ok %d", srv.response.success);
        }else {
            ROS_INFO("Failure to Arm");
            ROS_INFO("Exiting Program");
            return -1;
        }
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
    }

    
    // don't do anything while taking off
    double height_limit;
    height_limit = srv_takeoff.request.altitude - .1;
    ROS_INFO("Height Limit Set to %f", height_limit);
    
    while(loc_pos.pose.position.z < height_limit){
    	ros::spinOnce();
    	//sleep(1);
    	//ROS_INFO("Current Height of the UAV %f", loc_pos.pose.position.z);
    } 

    ROS_INFO("Takeoff Complete");



    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    

    /////////////////////////////////////////////////////////
    ///////////////Check Attitude Control/////////////
    /////////////////////////////////////////////////////////
    tf2::Quaternion Q;
    geometry_msgs::PoseStamped angle_command;
    ros::Publisher orient_pub;
    orient_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
    float roll;

    roll = 3.14/6;
    Q.setRPY(roll,0, 3.14); // roll, pitch, yaw
    


    ROS_INFO("the roll is %f", roll);
    angle_command.pose.orientation.x = Q[0];
    angle_command.pose.orientation.y = Q[1];
    angle_command.pose.orientation.z = Q[2];
    angle_command.pose.orientation.w = Q[3];
    orient_pub.publish(angle_command);

    ROS_INFO("Start Sleep");
    sleep(30);
    ROS_INFO("End Sleep");

    /////////////////////////////////////////////////////////////////////
    ////////////////////Fly to different spot///////////////////////////
    ////////////////////////////////////////////////////////////////////

    //setpoint 1
    double dif_x;
    double dif_y;
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 5;
    setpoint.pose.position.y = 10;
    ros::spinOnce(); //pickup the current height
    setpoint.pose.position.z = loc_pos.pose.position.z;
    
    loc_pos_pub.publish(setpoint);

    ROS_INFO("We Are Moving to Setpoint 1");
    
    dif_x = 100;
    dif_y = 100;
    while(dif_x >.1 && dif_y >.1) {
        ros::spinOnce(); //don't do anything while moving
        dif_x = abs(setpoint.pose.position.x-loc_pos.pose.position.x);
        dif_y = abs(setpoint.pose.position.y-loc_pos.pose.position.y);
    }


    //setpoint 2
    setpoint.pose.position.x = -5;
    setpoint.pose.position.y = -4;
    ros::spinOnce(); //pickup the current height
    setpoint.pose.position.z = loc_pos.pose.position.z;
    
    loc_pos_pub.publish(setpoint);

    ROS_INFO("We Are Moving to Setpoint 2");
    
    dif_x = 100;
    dif_y = 100;
    while(dif_x >.1 && dif_y >.1) {
        ros::spinOnce(); //don't do anything while moving
        dif_x = abs(setpoint.pose.position.x-loc_pos.pose.position.x);
        dif_y = abs(setpoint.pose.position.y-loc_pos.pose.position.y);
    }    

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
    	ROS_INFO("Landing");
       // ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }

    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
