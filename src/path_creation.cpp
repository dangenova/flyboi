#include <cstdlib>
#include <math.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseStamped.h>


// Initialize Publisher
ros::Publisher path_pub;
geometry_msgs::PoseStamped path;


// define path calculation function
void path_create(double t) {
	
	double x, y, z, a, b;
	a = 1.0;
	b = 0.8;

	x = a;
	y = 5;
	z = 5;

	path.pose.position.x = x;
	path.pose.position.y = y;
	path.pose.position.z = z;
	path_pub.publish(path)
}


//Main

int main(int argc, char** argv) {

	ros::init(argc, argv, "path_creation");
	ros::NodeHandle nh;
	

	path_pub = nh.advertise<geometry_msgs::PoseStamped >("/firefly/command/pose", 10);

	while (n.ok()) {


  }

}