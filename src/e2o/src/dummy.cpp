#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <math.h>
#include "e2o/input.h"

using namespace std;

#define lr (float) 1.15
#define lf (float) 0.75

e2o::input cmd_msg;


int n; // the number of sections to be used in the array 


float vel, steering_ang,tim;  
//float v_d_theta [1000][3];
bool update_flag=1;
float vel_update=0;





int main(int argc, char **argv)
{

  int flag=0; 

  // cout<< "please enter the number of sections of the path to be traced:"<<endl;
  // cin >> n;
  // cout<< " please enter the values of velocity distance and theta in the same order:" <<endl;
  // for(int i=0; i<n;i++)
  // {
  //   for(int k=0;k<3;k++)
  //   {
  //     cin >> v_d_theta[i][k];
  //   }
  
  // }
  // cout<<"the values have been input"<<endl;
 // for(int i=0; i<3;i++)
 //  {
 //    for(int k=0;k<n;k++)
 //    {
 //      cout<<v_d_theta[i][k]<<endl;
 //    }
  
 //  }

  // calc_time();
  // calc_vel();
  // calc_delf();

// for (int j=0; j<n; j++)
// {
//   cout<<vel[j]<<endl;
//   cout<<steering_ang[j]<<endl;
//   cout<<tim[j]<<endl;

// }
  
  ros::init(argc, argv, "dummy");

  ros::NodeHandle nh;

  ros::Publisher cmd_pub  = nh.advertise<e2o::input>("cmd_msg", 1000);

  ros::Rate loop_rate(2);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   
  // for(int p=0; p < n; p++)
  // {
  //   traj_msg.velocity.push_back (vel[p]);//{vel[0],vel[1],vel[2],vel[3]};
  //   traj_msg.steering_angle.push_back (steering_ang[p]);
  //   traj_msg.time.push_back (tim[p]);
  //   traj_msg.sections=n;
  
  // }
     

     cmd_msg.velocity = 4;
     cmd_msg.steering_angle = 0;
     cmd_msg.time = 2;
     cmd_msg.mode = 1;



  while (ros::ok())
  {
    // if (flag<50)
  
    
   
      cmd_pub.publish(cmd_msg);
  

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
