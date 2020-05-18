#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using namespace std;

double x_pos,y_pos,theta;
double goal_x,goal_y;
geometry_msgs::Twist vel_cmd;
std_msgs::Bool node_status;

double deg2rad(double ang)
{
	return (ang*M_PI);
}

double rad2deg(double ang)
{
  return(ang*180/M_PI);
}

void pose_callback(nav_msgs::Odometry odom_msg)
{
		y_pos = odom_msg.pose.pose.position.y;
		x_pos = odom_msg.pose.pose.position.x;
		theta = odom_msg.pose.pose.orientation.z;
}

void waypoint_callback(geometry_msgs::PointStamped waypt_msg)
{
	goal_x = waypt_msg.point.x;
	goal_y = waypt_msg.point.y;
	cout<<goal_x<<' '<<goal_y<<'\n';
}

double ang_error()
{
	double angle_diff;
	double x_diff = goal_x - x_pos;
	double y_diff = goal_y - y_pos;
	angle_diff = atan2(y_diff,x_diff) - theta;

	return angle_diff;
}

void node_callback(std_msgs::Bool node_msg)
{
	node_status.data = node_msg.data;
}
double dist()
{
	return sqrt((x_pos - goal_x)*(x_pos -goal_x) + (y_pos - goal_y)*(y_pos - goal_y));
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"follower");
	
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

	ros::Subscriber pose_sub = n.subscribe("/odometry/filtered",1,pose_callback);
	ros::Subscriber goal_sub = n.subscribe("/waypoints_map",10,waypoint_callback);
	ros::Subscriber node_stat = n.subscribe("/node_status",10,node_callback);

	ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",100);
	ros::Publisher way_reach = n.advertise<std_msgs::Bool>("/waypoint_reached",10);

	double ang_diff=0;
	int pubRate = 10;
	ros::Rate rate(pubRate);
	bool rotate = false;
	std_msgs::Bool reach;
	reach.data = false;

	while(ros::ok())
	{
		if(dist() <= 0.7)
		{
			reach.data = true;
			way_reach.publish(reach);
		}
		if(node_status.data == true)
			break;
		rotate = false;
		ang_diff = rad2deg(ang_error());
		cout<<"Angle :"<<ang_diff<<'\n';
		if(fabs(ang_diff) > 2)
			rotate = true;

		if(rotate == true)
		{
			vel_cmd.linear.x = 0.5;
			if(ang_diff > 0)
				vel_cmd.angular.z = 0.2;
			else 
				vel_cmd.angular.z = -0.2;
		}
		else
		{
			vel_cmd.linear.x = 1.0;
			vel_cmd.angular.z = 0.0;
		}
		pub_vel.publish(vel_cmd);
		ros::spinOnce();
		rate.sleep();
	}
	vel_cmd.linear.x = 0.0;
	vel_cmd.angular.z = 0.0;

	pub_vel.publish(vel_cmd);

	return 0;
}