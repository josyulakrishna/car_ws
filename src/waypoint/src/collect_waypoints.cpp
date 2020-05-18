#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
// #include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>

/*
#include "robot_localization/navsat_conversions.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "sensor_msgs/NavSatFix.h"
*/



using namespace std;
long double lat,lon;

std::string utm_zone;
geometry_msgs::PointStamped UTM_point, map_point, prev_point;

float c_dist;

void gps_callback(sensor_msgs::NavSatFix gps_msg)
{
	lat = gps_msg.latitude;
	lon = gps_msg.longitude;
	// cout<<gps_msg.latitude<<' '<<gps_msg.longitude<<'\n';
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

int main(int argc, char** argv)
{
	if(argc < 1)
	{
		cout<<"1. Enter waypoint collection distance (50)"<<endl;
		return -1;
	}

	c_dist = atof(argv[1]);

	ros::init(argc,argv,"collect_waypoints");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

	ros::Subscriber gps_sub = n.subscribe("/gps/filtered",1,gps_callback);

	ofstream out_file("waypoints.txt");
	int a;
	int flag= 0;

	float dist = 0;

	prev_point.point.x = 0;
	prev_point.point.y = 0;
	while(ros::ok())
	{
	
		// out_file <<setprecision(12)<<lat<<' '<<setprecision(12)<<lon<<'\n';
		cout<<setprecision(12)<<lat<<' '<<setprecision(12)<<lon<<'\n';
		UTM_point = latLongtoUTM(lat, lon);
		map_point = UTMtoMapPoint(UTM_point);
		cout<<UTM_point.point.x<<' '<<UTM_point.point.y<<'\n';
		cout<<map_point.point.x<<' '<<map_point.point.y<<'\n';

		dist = sqrt((map_point.point.x - prev_point.point.x)*(map_point.point.x - prev_point.point.x) + (map_point.point.y - prev_point.point.y)*(map_point.point.y - prev_point.point.y));

		if(dist > c_dist)
		{
			out_file <<setprecision(5)<<lat<<' '<<setprecision(5)<<lon<<'\n';
			dist = 0;
			prev_point.point.x = map_point.point.x;
			prev_point.point.y = map_point.point.y;
		}
		
		
		ros::spinOnce();
	}

	out_file.close();

	return 0;

}
