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
#include <nav_msgs/Odometry.h>


using namespace std;

std::string utm_zone;
geometry_msgs::PointStamped UTM_point, map_point;
std_msgs::Bool is_reach;
std::vector<double> waypoints[2];

double x_pos,y_pos,theta;
double goal_x,goal_y;

//---------------- Read Waypoints from the file-----------//
void get_waypoints(string waypoints_file)
{
	std::string line;
	std::ifstream infile_waypoints(waypoints_file.c_str ());
	cout<<"------------ GPS waypoints ------------------\n";
	while (std::getline(infile_waypoints, line))
	{
		std::istringstream iss(line);
		float lat,lon;
		if (!(iss >> lat >> lon)) { break; } // error
		// add waypoints
		waypoints[0].push_back(lat);
		waypoints[1].push_back(lon);
		cout<<waypoints[0].size()<<" ";
		printf("%lf %lf " , lat, lon);
		cout<<"\n";
	}
	cout<<"------------------------------------------------\n";
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

void pose_callback(nav_msgs::Odometry odom_msg)
{
		y_pos = odom_msg.pose.pose.position.y;
		x_pos = odom_msg.pose.pose.position.x;
		theta = odom_msg.pose.pose.orientation.z;
}

// void waypoint_callback(std_msgs::Bool way_reach)
// {
// 	is_reach.data = way_reach.data;
// }

int main(int argc, char **argv)
{
	ros::init(argc,argv,"gps_waypoint");
	// if(argc<1)
	// {
	// 	cout<< "Enter the arguments : \n"
	// 			"1) Waypoints file\n"
	// 			"\n";
	// 	return -1;
	// }
	string txt = "points.txt";
	get_waypoints(txt);
	

	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

	
	ros::Publisher goal_pub = n.advertise<geometry_msgs::PointStamped>("/waypoints",1);
	// ros::Publisher node_pub = n.advertise<std_msgs::Bool>("/node_status",1);

	// ros::Subscriber wayReach_sub = n.subscribe("/waypoint_reached",10,waypoint_callback);
	ros::Subscriber pose_sub = n.subscribe("/odometry/filtered",1,pose_callback);



	int pubRate = 10;
	ros::Rate rate(pubRate);
	geometry_msgs::PointStamped goalp;

	int itr = 0;
	float dist = 0;

	while(itr < waypoints[0].size())
	{
		UTM_point = latLongtoUTM(waypoints[0][itr], waypoints[1][itr]);
		map_point = UTMtoMapPoint(UTM_point);

		goalp = map_point;
		dist = sqrt((map_point.point.x - x_pos)*(map_point.point.x - x_pos) + (map_point.point.y - y_pos)*(map_point.point.y - y_pos));

		if(dist < 3.0)
		{
			cout<<"Waypoint "<<itr<<" reached"<<endl;
			itr++;
			// dist = 0;
		}
		
		goal_pub.publish(goalp);
		
		rate.sleep();
		ros::spinOnce();

	}

	cout<<"Goal Reached"<<endl;
	return 0;
}
