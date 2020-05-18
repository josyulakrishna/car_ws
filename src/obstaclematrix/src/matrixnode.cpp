#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/pcl_base.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/crop_box.h"
#include "math.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/NavSatFix.h>
#include "nlgl.h"
#include "nlgl.cpp"
// #include <pixhwk_ctrl_node/Lanes.h>
// #include "../../PixHawkController/nlgl.h"
// #include "../../PixHawkController/nlgl.cpp"
#include <vector>
#include <nav_msgs/Odometry.h>


#define m_size (101)
// #define scale_factor (4)
// for size = 20 scale_factor = 1;
// factor = 1 means 1/1 meter, factor = 5 means 1/5 meter

// enter padding terms wrt to the center of the vehicle
int padding = 2;
double min_dev = 5;//+- 5 degrees error 
// padding = 1 means , 1/(scale_factor) meter padding per block. 5 means 5*(1/(scale_factor)) meter padding.
#define lidar_shift_right (1) // positive if right, amount that lidar is dispalced from the center of the vehicle in left right direction
#define lidar_shift_front (3) // positive if froward, amount that lidar is dispalced from the center of the vehicle in front back direction
int scale_factor = 2;

using namespace std;


static double rotation[3] = {0.0, 0.0, 0.0,};

int loop_counter = 0;
static int Matrix[m_size][m_size] ={0};
nav_msgs::OccupancyGrid map123;

//these are in lidar
double heading = 0;
static double my_position[2] = {0,0};
// static double my_goal[3];

static int reached_goal = 0; //if 1 then goal has been reached else 0
int min_distance_for_goal = 2; //no of cells
int max_distance_goal = 10; //max goal length
static int goal_x = 0;
static int goal_y = 0;
int lidar_x = (m_size+1)/2;
int lidar_y = (m_size+1)/2;
int home_x = (m_size+1)/2;// + lidar_shift_right;
int home_y = (m_size+1)/2;// - lidar_shift_front;

double goal_heading = 0;

double x_pos = 0,y_pos = 0;
ros::Publisher map_pub;

/*
obstacle is 1
clear is 0
*/
///////////////////////////////////////////////////////////////
struct Point
{
  int x;
  int y;
};

vector<double> left_lane_points[2];
vector<double> right_lane_points[2];

/*
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
*/

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise

/*
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
*/

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
/*
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{	
	// cout<<"Inside doIntersect \n";
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}
*/

// Returns true if the point p lies inside the polygon[] with n vertices
/*
bool isInside(Point p, Point polygon[])
{
	// cout<<"Inside isInside \n";
    // Create a point for line segment from p to infinite
    Point extreme;
    extreme.x=0;
    extreme.y=0;

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%4;

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
               return onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);
    // cout<<"Outside isInside \n";
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}

bool is_inside_lane(Point p) // Checks whether the given point is whithin the lane
{
	// cout<<"Inside is_inside_lane \n";

	bool status = false;
	Point polygon[4];
	int l=0,r=0;
	// cout<<"Lane instant size :"<<left_lane_points[0].size()<<' '<<right_lane_points[0].size()<<'\n';
	while((l+1)<(left_lane_points[0].size()-1) || (r+1)<(right_lane_points[0].size()-1))
	{
		// cout<<"inside while \n";
		polygon[0].x=left_lane_points[0][l];
		polygon[0].y=left_lane_points[1][l];
		polygon[1].x=right_lane_points[0][r];
		polygon[1].y=right_lane_points[1][r];
		if((r+1)<right_lane_points[0].size())
			r=r+1;
		if((l+1) < left_lane_points[0].size())
			l=l+1;
		polygon[2].x = right_lane_points[0][r];
		polygon[2].y = right_lane_points[1][r];
		polygon[3].x = left_lane_points[0][l];
		polygon[3].y = left_lane_points[1][l];

		status = isInside(p,polygon);
		if(status == true)
			break;
	}
	// polygon[0].x=70;
	// polygon[0].y = 0;
	// polygon[1].x = 30;
	// polygon[1].y = 0;
	// polygon[2].x = 30;
	// polygon[2].y = 100;
	// polygon[3].x = 70;
	// polygon[3].y = 100;

	// status = isInside(p,polygon);
	return status;
	// cout<<"Leaving is_inside_lane \n";
}
*/
//////////////////////////////////////////////////////////////////////

int get_goal_quadrant(double heading)
{
	int goal_quadrant = 0;

	if(heading>0 && heading<=90)
		goal_quadrant=0;		
	else if(heading>90 && heading<=180)
		goal_quadrant=1;
	else if(heading<0 && heading>=-90)
		goal_quadrant=2;
	else if(heading<-90 && heading>=-180)
		goal_quadrant=3;

	return goal_quadrant;
}

int get_local_quadrant(double heading)
{
	int local_quadrant = 0;

	if(heading>0 && heading<=90)
		local_quadrant=0;		
	else if(heading>90 && heading<=180)
		local_quadrant=1;
	else if(heading<0 && heading>=-90)
		local_quadrant=2;
	else if(heading<-90 && heading>=-180)
		local_quadrant=3;

	return local_quadrant;
}


void goal_heading_callback(const std_msgs::Float64::ConstPtr& heading_msg)
{
	heading = heading_msg->data;
	/*
	shift angle to accomodate for change in axis.
	FROM GPS to occupancy grid
	*/
	heading = heading;

	// heading = 45;
}
//int count_lanes_callback=0;
/*void lanes_callback(const pixhwk_ctrl_node::Lanes::ConstPtr& lanes_msg)
{
	// cout<<"lanes_callback\n";
	count_lanes_callback++;
	if(count_lanes_callback<4)
	{
		// count_lanes_callback =0;
		return;
	}
	else
		count_lanes_callback=0;
	left_lane_points[0].clear();
	right_lane_points[0].clear();
	left_lane_points[1].clear();
	right_lane_points[1].clear();
	double x,y;
	for(int i=0; i<lanes_msg->lane_left_points.size(); i++)
	{
		x = lanes_msg->lane_left_points[i].x*scale_factor;
		y = lanes_msg->lane_left_points[i].y*scale_factor;
		x = x + (m_size+1)/2;
		y = y + (m_size+1)/2;
		left_lane_points[0].push_back(x);
		left_lane_points[1].push_back(y);
	}
	for(int i=0; i<lanes_msg->lane_right_points.size(); i++)
	{	
		x = lanes_msg->lane_right_points[i].x*scale_factor;
		y = lanes_msg->lane_right_points[i].y*scale_factor;
		x = x + (m_size+1)/2;
		y = y + (m_size+1)/2;
		right_lane_points[0].push_back(x);
		right_lane_points[1].push_back(y);
	}


	cout<<"lanes Left size:"<<left_lane_points[0].size()<<'\n';
	cout<<"Lanes Right size:"<<right_lane_points[0].size()<<'\n';

}*/
void get_goal()
{
	cout<<"---------------heading : "<<heading<<"\n";
	// cout<<"0"<<" "<<"0"<<"  :  "<<rad2deg(atan2(0 - home_x, 0 - home_y))<<"\n";
	// cout<<"0"<<" "<<"50"<<"  :  "<<rad2deg(atan2(0 - home_x, 50 - home_y))<<"\n";
	// cout<<"0"<<" "<<"100"<<"  :  "<<rad2deg(atan2(0 - home_x, 100 - home_y))<<"\n";
	// cout<<"50"<<" "<<"0"<<"  :  "<<rad2deg(atan2(50 - home_x, 0 - home_y))<<"\n";
	// cout<<"50"<<" "<<"50"<<"  :  "<<rad2deg(atan2(50 - home_x, 50 - home_y))<<"\n";
	// cout<<"50"<<" "<<"100"<<"  :  "<<rad2deg(atan2(50 - home_x, 100 - home_y))<<"\n";
	// cout<<"100"<<" "<<"100"<<"  :  "<<rad2deg(atan2(100 - home_x, 100 - home_y))<<"\n";
	// cout<<"100"<<" "<<"50"<<"  :  "<<rad2deg(atan2(100 - home_x, 50 - home_y))<<"\n";
	// cout<<"100"<<" "<<"0"<<"  :  "<<rad2deg(atan2(100 - home_x, 0 - home_y))<<"\n";

	/*							(0)
								[50,100]
								|
								|
								|
	(90)[100,50]-------------[50,50]----------------[0,50](-90)
								|
								|
								|
								[50,0]
								(180,-180)
	*/
	double min_heading_diff = 10000;//some large value
	double final_local_heading = 100;
	int calcflag = 0;
	// for(int a = 51; a >= 40; a--)
	// {
	// 	for (int b = 90; b < 91; ++b)//select goal only in first two quadrants
	// 	{
	// 		double local_heading = fabs(90-rad2deg(atan2(a - home_x, b - home_y)));
	// 		// double local_distance = sqrt((a - home_x)*(a - home_x) + (b - home_y)*(b - home_y));
	// 		if(Matrix[a][b]==0 && Matrix[a+1][b]==0 && Matrix[a+2][b]==0)//if cell is free
	// 		{
	// 			// if(local_heading <= final_local_heading)
	// 			// {
	// 			// 	final_local_heading = local_heading;
	// 			// 	goal_x = a;
	// 			// 	goal_y = b;
	// 			// }
	// 			goal_x = a;
	// 			goal_y = b;
	// 			calcflag = 1;
	// 			break;
				

 // //     //            if(!flag_goal)
	// // 				// if(fabs(local_heading - heading)<= min_heading_diff)
	// // 				// {
	// // 				// 	if(local_distance >= min_distance_for_goal && local_distance <= max_distance_goal)
	// // 				// 	{
	// // 				// 		if(get_goal_quadrant(heading) == get_local_quadrant(local_heading))
	// // 				// 		{
	// // 				// 			min_heading_diff = fabs(local_heading - heading);
	// // 				// 			goal_x = a;
	// // 				// 			goal_y = b;
	// // 				// 			final_local_heading = local_heading;
	// // 				// 		}
	// // 				// 	}
	// // 				// }
	// 		}

	// 	}
	// 	if(calcflag ==1)
	// 		break;
	// }

	float heading_diff;
	for(int a = 0; a< m_size; a++)
	{
		for(int b = 95; b < m_size; b++)
		{
			double local_heading = -rad2deg(atan2(a - home_x, b - home_y));
			if(Matrix[a][b]==0 && Matrix[a][b-1]==0 && Matrix[a+1][b]==0 && Matrix[a-1][b]==0)
			{

				heading_diff = fabs(local_heading - goal_heading);

				cout<<a<<' '<<b<<' '<<heading_diff<<' '<<local_heading<<' '<<heading<<endl;
				
				if(heading_diff < min_heading_diff)
				{
					min_heading_diff = heading_diff;
					goal_x = a;
					goal_y = b;

				}

			}
		}
	}
	cout<<"---------------local heading : "<<min_heading_diff<<"\n";
	// if(min_heading_diff > min_dev)
	// {
	// 	cout<<"++++++MIN heading : "<<min_heading_diff<<"\n";
	// 	//if cannot find heading just move forward
	// 	goal_x = home_x;
	// 	goal_y = home_y + 5;
	// }
}

void obstacle_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& msg)
{
	loop_counter++;
	if(loop_counter%1 == 0)
	{	
		loop_counter = 0;
		
		//build occupancy grid
		std::vector<signed char> v((m_size)*(m_size));
		pcl::PointXYZRGBL p;
		for(int i = 0; i < msg->width; i++)
		{	
			p = msg->points[i];
			// scaling the coordinates
			p.x = scale_factor*p.x;
			p.y = scale_factor*p.y;
			p.z = scale_factor*p.z;

			//translation
			int m = p.x + (m_size+1)/2;
			int l = p.y + (m_size+1)/2;
			
			//filling up the obstacle matrix
			if((l >= padding && m >= padding) && (l < (m_size - padding) && m < (m_size - padding)))
			{
				for (int a = (-padding); a <= padding; ++a)
				{
					for (int b = (-padding); b <= padding; ++b)
					{
						if (l != home_y && m != home_x)
						{
							v[(l+a)*m_size + (m+b)] = 100;
						}
						v[(home_x+a)*m_size + (home_y+b)] = 50;
					}
				}
			}
		}
		// Checks whether the points lie whitin the lane or not
		// int flag_obs = 0;
		// Point test;
		// if(!(left_lane_points[0].size()>2) || !(right_lane_points[0].size()>2))
		// 	flag_obs=1;
		// for (int a = 0; a < m_size; ++a)
		// {	
		// 	if(flag_obs == 1)
		// 		break;
		// 	for (int b = 0; b < m_size; ++b)
		// 	{
		//         test.x = a;
		//         test.y = b;
		// 		if (!is_inside_lane(test))
		// 		{
  // 		// @d - writing in terms of the 2d matrix with row (l+a)
		// 			v[a*m_size + b] = 100;
		// 		}
		// 	}
		// }
		// for(int q=0;q<left_lane_points[0].size();q++)
		// {
		// 	if(left_lane_points[0][q] >=0 && left_lane_points[0][q] < m_size-1)
		// 		if(left_lane_points[1][q] >=0 && left_lane_points[1][q] < m_size-1)
		// 			v[left_lane_points[0][q]*m_size + left_lane_points[1][q]] = 50;
		// }
		// for(int q=0;q<right_lane_points[0].size();q++)
		// {
		// 	if(right_lane_points[0][q] >=0 && right_lane_points[0][q] < m_size-1)
		// 		if(right_lane_points[1][q] >=0 && right_lane_points[1][q] < m_size-1)
		// 			v[right_lane_points[0][q]*m_size + right_lane_points[1][q]] = 70;
		// }

		for (int i = 0; i < m_size; ++i)
		{
			for (int j = 0; j < m_size; ++j)
			{
				if(j<(m_size-8)/2)
				{
					v[i*m_size + j] = 100;
					Matrix[i][j] = 1;
				}
				if (v[i*m_size + j] == 100)
				{
					Matrix[i][j] = 1;//occupied
				}
				else
				{
					Matrix[i][j] = 0;//free
				}
			}
		}



		//Deciding the local goal on the occupancy grid with respect to the velodyne map
		get_goal();

		// goal_x = 51;
		// goal_y = 100;


		//ROS_INFO("Home(x,y): %d %d",home_x,home_y);
		//ROS_INFO("Goal(x,y): %d %d",goal_x,goal_y);
	
		// for visualising home postion, lidar postion and goal position on rviz
		v[home_x*m_size + home_y] = 10;
		v[(lidar_x)*m_size + (lidar_y)] = 30;
		// v[goal_x*m_size + goal_y] = 90;

		//creating the text file with goal and home coordinates
		FILE *g = fopen("/home/rrc/swahana_ws/goal.txt", "w");

		if (g == NULL)
		{
		    printf("Error opening file!\n");
		    exit(1);
		}
		fprintf(g, "%d %d\n", home_x,home_y);
		fprintf(g, "%d %d", goal_x,goal_y);
		fclose(g);
		cout<<"Goal :"<<goal_x<<' '<<goal_y<<'\n';

		//Creating the text file from occupancy matrix


		FILE *f = fopen("/home/rrc/swahana_ws/matrix.txt", "w");
		if (f == NULL)
		{
		    printf("Error opening file!\n");
		    exit(1);
		}
		fprintf(f, "%d %d\n", m_size,m_size);

		
		for(int a = 0; a < (m_size); a++){
			for (int b = 0; b < (m_size); ++b)
			{
				fprintf(f, "%d ", Matrix[a][b]);
			}
			if(a != m_size-1)
				fprintf(f, "\n");
		}
		fclose(f);

		// Launching the Ompl script
		if(reached_goal == 0)
			system("/home/rrc/swahana_ws/Ompl.sh");
			// int efg = 0;
		else
		{
			FILE *f = fopen("/home/rrc/swahana_ws/trajectory_ompl.txt", "w");
			if (f == NULL)
			{
			    printf("Error opening file!\n");
			    exit(1);
			}
			fprintf(f, "0 0\n");
			fclose(f);
		}

		double InputX,InputY;
		ifstream iFile;
		iFile.open("/home/rrc/swahana_ws/trajectory_ompl.txt");
		double ang;
		int octr = 0;
		while((iFile >> InputX) && (iFile >> InputY) && octr < 3)
		  {
		    v[int(InputX)*(m_size) + int(InputY)] = 70;
		    // octr++;
		  }

		// InputX = 51;
		// InputY = 53;
		//   while(octr < 3)
		//   {
		//     v[int(InputX)*(m_size) + int(InputY)] = 70;
		//     octr++;
		//     InputY = InputY + 2;
		//   }

		iFile.close();

		map123.data = v;
		map123.header.frame_id = "map";
		map123.info.origin.orientation.w = 1;
		map_pub.publish(map123);
	}
}

void odom_callback(nav_msgs::Odometry odom_msg)
{
		y_pos = odom_msg.pose.pose.position.y;
		x_pos = odom_msg.pose.pose.position.x;
		heading = rad2deg(odom_msg.pose.pose.orientation.y);

		// goal_heading = rad2deg(atan2((goal_map_x - y_pos),(goal_map_x - x_pos))) - heading;
		goal_heading = heading;

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "matrix_node");
	ros::NodeHandle n;

	if (argc < 2)
	{
		cout << "Enter the following arguments ..\n"
            "1) padding (2)\n"
            "2) min deviation from heading (5.0)\n"
            "3) min distance for goal (2)\n"
            "4) max distance for goal (10)\n"
            "4) scale_factor (2)\n"
            "\n";
        return -1;
	}

  padding = atoi(argv[1]);
  min_dev = atof(argv[2]);
  min_distance_for_goal = atoi(argv[3]);
  max_distance_goal =  atoi(argv[4]);
  scale_factor =  atoi(argv[5]);

  map123.info.resolution = 1;
  map123.info.width = m_size;
  map123.info.height = m_size;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  ros::Subscriber obstacle_sub = n.subscribe("/velodyne_obstacles",1,obstacle_callback);
  ros::Subscriber pos_sub = n.subscribe("/integrated_to_init",1,odom_callback);

  // ros::Subscriber goal_sub = n.subscribe("/waypoints",1,goal_callback);
  // ros::Subscriber goal_sub = n.subscribe("/odometry/filtered",1,odom_callback);

  // ros::Subscriber heading_sub = n.subscribe("/car_heading",1,goal_heading_callback);
  // ros::Subscriber lanes_sub = n.subscribe<pixhwk_ctrl_node::Lanes>("/lanes",1,lanes_callback);

  while (ros::ok())
  {
  	ros::spin();
  }

  return 0;
}
