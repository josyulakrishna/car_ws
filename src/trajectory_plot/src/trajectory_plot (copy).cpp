#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <math.h>
#include "trajectory_plot/input.h"

using namespace std;

#define lr (float) 1.15
#define lf (float) 0.75

trajectory_plot::input traj_msg;


int n; // the number of sections to be used in the array 


float vel[1000], steering_ang[1000],tim[1000];  
float v_d_theta [1000][3];
bool update_flag=1;
float vel_update=0;


void calc_time(void)
{ 
  for(int i=0; i<n; i++)
  {
    tim[i]= v_d_theta[i][1]/v_d_theta[i][0];
  }
}

void calc_delf(void)
{

  for (int i=0; i<n; i++)
  {
    steering_ang[i]= (float) atan(((lr+lf)/lr)*tan(asin((lr*v_d_theta[i][2])/(tim[i]*v_d_theta[i][0]))));
    steering_ang[i]= (float) steering_ang[i]*(180/3.1415)*(135/35);
  }

}

void calc_vel(void)
{
  for (int i=0; i<n; i++)
  {
    vel[i]=v_d_theta[i][0];
  }

  // if(vel_update!=0)
  // {
  // 	update_flag=0;
  // }
  // else
  // {
  // 	update_flag=1;
  // }

  // vel_update=vel[0];

}




int main(int argc, char **argv)
{

  int flag=0; 

  cout<< "please enter the number of sections of the path to be traced:"<<endl;
  cin >> n;
  cout<< " please enter the values of velocity distance and theta in the same order:" <<endl;
  for(int i=0; i<n;i++)
  {
    for(int k=0;k<3;k++)
    {
      cin >> v_d_theta[i][k];
    }
  
  }
  cout<<"the values have been input"<<endl;
 // for(int i=0; i<3;i++)
 //  {
 //    for(int k=0;k<n;k++)
 //    {
 //      cout<<v_d_theta[i][k]<<endl;
 //    }
  
 //  }

  calc_time();
  calc_vel();
  calc_delf();

for (int j=0; j<n; j++)
{
  cout<<vel[j]<<endl;
  cout<<steering_ang[j]<<endl;
  cout<<tim[j]<<endl;

}
  
  ros::init(argc, argv, "trajectory_plot");

  ros::NodeHandle nh;

  ros::Publisher traj_pub  = nh.advertise<trajectory_plot::input>("mlc_traj_control", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   
  for(int p=0; p < n; p++)
  {
    traj_msg.velocity.push_back (vel[p]);//{vel[0],vel[1],vel[2],vel[3]};
    traj_msg.steering_angle.push_back (steering_ang[p]);
    traj_msg.time.push_back (tim[p]);
    traj_msg.sections=n;
  
  }
     


  while (ros::ok())
  {
    // if (flag<50)
  
    
    if (flag<100 && update_flag==1)
    {
    	traj_pub.publish(traj_msg);
    	
    }

    flag=flag+1;
    

  

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
