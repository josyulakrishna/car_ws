#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
// #include <e2o/input.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

ros::Time st_time;
ros::Time temp_time;

ofstream file;

double vel = 0;
int f = 0;

void topic_cb(geometry_msgs::Twist cmd_vel)
{
	if(f == 0)
	{
		st_time = ros::Time::now();
		f=1;
		return;
	}
	vel = cmd_vel.linear.x;
	cout<<cmd_vel.linear.x;
	temp_time= ros::Time::now();

	file<<vel<<' '<<(temp_time - st_time)<<endl;

	cout<<' '<<(temp_time - st_time)<<endl;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_write");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  file.open("straight.txt");


  ros::Subscriber topic_sub = n.subscribe("/cmd_vel",1,topic_cb);

  

  while(ros::ok())
  {
  	ros::spinOnce();
  }
  file.close();
  return 0;
}
