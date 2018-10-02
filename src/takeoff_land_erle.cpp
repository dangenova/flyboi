#include <cstdlib>
#include <math.h>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseStamped loc_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	loc_pos = *msg;
}

int main(int argc, char **argv)
{

    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff_erle");
    ros::NodeHandle n;

    ros::Rate r(rate);

	ros::Subscriber loc_pos_sub = n.subscribe("/mavros/local_position/pose", 10, pose_cb);
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
    	ROS_INFO("Armed");
       // ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
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
