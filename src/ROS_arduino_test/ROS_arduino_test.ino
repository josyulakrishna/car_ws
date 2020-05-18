#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <trajectory_plot/input.h>

ros::NodeHandle nh;

trajectory_plot::input stat_pack;
ros::Publisher pub_val("vehicle_stat", &stat_pack);

long int enc_dif=100;

void setup()
{
  nh.initNode();
  nh.advertise(pub_val);
}

void loop()
{

  stat_pack.sections=enc_dif;
  pub_val.publish(&stat_pack);
  nh.spinOnce();
  delay(1000);
  enc_dif=enc_dif-1;

  if(enc_dif==0)
  {
    enc_dif=100;
  }
}

