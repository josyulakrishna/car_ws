#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <vector>

ros::Publisher vel_pub;

double min_dist_bearing; // min bearing update distance
double min_dist_goal;   //min distance to between waypoint and car to consider it as reached goal
double current_gps_bearng = 0.0; 
double goal_gps_bearng = 0.0;
double current_pos[2] = {0,0};
double prev_pos[2] = {0,0};
vector<double> waypoints[2]; // waypoints vector [0] contains x and [1] contains y coordinates
double turn_angle=0.0;
int curr_goal_index = 0;

double angle_unwrap(double theta)
{
    /*
    going from (-180,180) to (0,360)
    */
    if(theta < 0)
    {
        theta = 360 + theta;
    }
    return theta;
}

double deg2rad(double ang)
{
	return (ang*M_PI);
}

double rad2deg(double ang)
{
  return(ang*180/M_PI);
}

double distance(double lat1, double lon1, double lat2, double lon2)
{
	double dlon = deg2rad(lon2 - lon1);
	double dlat = deg2rad(lat2 - lat1);
	double a = sin(dlat/2)*sin(dlat/2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)*sin(dlon/2);
	a = 2*atan2( sqrt(a), sqrt(1-a));
	double dist = 6371000*a;

	return dist;
}

double bearing(double lat1, double lon1, double lat2, double lon2)
{
	lat1 = deg2rad(lat1);
	lat2 = deg2rad(lat2);
	lon1 = deg2rad(lon1);
	lon2 = deg2rad(lon2);

	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
	double y = sin(lon2-lon1)*cos(lat2);
	double bearingValue = atan2(y,x);

	return bearingValue;
}
// ------------------- Calculates heading diff between goal and car--------//
double heading_shift(double current_gps_bearng, double goal_gps_bearng)
{
    double delta = angle_unwrap(goal_gps_bearng) - angle_unwrap(current_gps_bearng);
    /*
    delta +ve is right 
    delta -ve is left
    */
    double heading_shift = delta;
    double sum=0;
    double avg_heading=0;
    // heading_vect.push_back(heading_shift);
    // for(int i=0;i<heading_vect.size();i++)
    // { 
    //   sum = sum + heading_vect[i];
    // }
    // avg_heading = sum/heading_vect.size() ; // Smoothes out heading
    // if(heading_vect.size() == smooth_size)  
    //   heading_vect.erase(heading_vect.begin()); // Pops back first element
    // cout<<"current bearing : "<< current_gps_bearng<<"\n";
    // cout<<"goal bearing    : "<< goal_gps_bearng<<"\n";
    if(delta > 0)
        cout<<"right : "<<delta<<"\n";
    else
        cout<<"left  : "<<delta<<"\n";

    /*
    delta -ve is right
    delta +ve is left
    */

    return -1.0*delta;
}
int flag = 0;
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	if(gps_msg->status.status == 0)
	{
		if(flag == 0)
        {
            prev_position[0] = gps_msg->latitude;
            prev_position[1] = gps_msg->longitude;
            flag = 1;
            return;
        }
		current_pos[0] = gps_msg->latitude;
		current_pos[1] = gps_msg->longitude;

		current_goal[0] = waypoints[0][curr_goal_index];
		current_goal[1] = waypoints[1][curr_goal_index];

		double dist = distance(prev_position[0], prev_position[1], current_position[0], current_position[1]);
		
		if(dist > min_dist_bearing)
        {

            current_gps_bearing = rad2deg(bearing(prev_position[0], prev_position[1], current_position[0], current_position[1]));
            prev_position[0] = gps_msg->latitude;
            prev_position[1] = gps_msg->longitude;

            goal_gps_bearing = rad2deg(bearing(current_position[0], current_position[1], current_goal[0], current_goal[1]));

            double distance_to_goal = distance(current_position[0], current_position[1], current_goal[0], current_goal[1]);
            turn_angle = deg2rad(heading_shift(current_gps_bearing,goal_gps_bearing));
    		
    		if(distance_to_goal < min_dist_goal)
    		{
    			if(curr_goal_index >=0 && curr_goal_index < waypoints[0].size())
    			{
    				cout<<"Waypoint Reached!!";
    				curr_goal_index++;
    			}
    			else
    			{
    				cout<<"Goal Reached!! \n";
    			}

    		}
        }
	}
}

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

int main(int argc, char **argv)
{
	ros::init(argc,argv,"waypoint");
	if(argv<1)
	{
		cout<< "Enter the arguments : \n"
				"1) Waypoints file\n"
				"\n";
		return -1;
	}

	get_waypoints(string(argv[1]));
	
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

	ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("/gps/fix",1000,gps_callback,noDelay);
	
	vel_pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",1);

	while(ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}
