#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <e2o/input.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

#define lf (float)0.75
#define lr (float)1.15

float steering_command,steering_ratio;
int my_position[2];
float vel;

e2o::input e2o_cmd;

double ang_wrap(double ang)
{
  if (ang <= -1 * M_PI)
    ang = ang + 2*M_PI;


  if (ang >= M_PI)
    ang = ang - 2*M_PI;

  return(ang);
}

// converting degrees to radians
double deg2rad(double ang)
{
  return(ang*M_PI/180);
}

// converting radian to  degrees
double rad2deg(double ang)
{
  return(ang*180/M_PI);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  if(argc < 1)
  {
    cout<<"1) Steering ratio (1)\n"; // Hyper Parameter
    exit(1);
   } 
  steering_ratio = atof(argv[1]);
  ros::NodeHandle private_nh("~");
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Publisher pub_msg;
  
  pub_msg = n.advertise<e2o::input>("/cmd_msg", 1);

  ros::Rate r(2);

  while(ros::ok())
  {

    double x,y,theta;
    ifstream iFile;

    iFile.open("/home/rrc/swahana_ws/trajectory_ompl.txt");
	
	  my_position[0] = 51;
  	my_position[1] = 51;
  	vel = 3;
    
    int ctr = 0;
    while((iFile >> x) && (iFile >> y))
      {
        ctr++;
        if(ctr == 3) // Read the third point
        {
          cout<<"Goal : "<<x<<' '<<y<<'\n';
          // Calculate Normalized Steering and multiply by steering ratio.
          steering_command = ((90-(rad2deg(atan2(y - my_position[1], x - my_position[0]))))*steering_ratio);
          // theta = (90-(rad2deg(atan2(y - my_position[1], x - my_position[0]))));
          // steering_command = atan(((lr+lf)/lr)*tan(asin((lr*vel))))
          // cout<<"Steering Ratio : "<<steering_ratio<<endl;
          // cout<<"original steering "<<90-rad2deg(atan2(y - my_position[1], x - my_position[0]))<<'\n';
          // ROS_INFO("goal = %lf,%lf",my_goal[0],my_goal[1]);
          // ROS_INFO("waypoint = %lf,%lf",x,y);  
          cout<<"converted steering : "<< steering_command;
          ctr = 0;
          break;
        }
      }

    if(steering_command >= 35)
      steering_command = 35;
    else if(steering_command <= -35)
      steering_command = -35;
    e2o_cmd.velocity = vel;
    e2o_cmd.steering_angle = steering_command;
    e2o_cmd.time = 1;
    e2o_cmd.mode = 1;

    pub_msg.publish(e2o_cmd);

    r.sleep();

    ros::spinOnce();


  }
return 0;
}
