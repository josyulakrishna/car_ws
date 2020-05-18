#include <math.h>
#include <ros/ros.h>
#include "nlgl.h"
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

// the first x,y value is the home coordinates
double home3_dx = 0;
double home3_dy = 0;
double goal_3dx = 0;
double goal_3dy = 0;
static int first_xy = 1;

double nlgl(double temp_uav_pos[2],  double kiang, double goal[2])
{
    return nlgl_3d_map(temp_uav_pos, kiang, goal);
}

double nlgl_3d_map(double temp_uav_pos[2], double kiang, double goal[2])
{
  goal_3dx = goal[0];
  goal_3dy = goal[1];
  //ROS_INFO("uavposition local coordinates:%f %f",temp_uav_pos[1],temp_uav_pos[2]);
  double u = 0;
  double goal_angle = 0;
  double consecutive_angle = 0;
  static double previous_x = 0;
  static double previous_y = 0;  
    
  goal_angle = atan2(goal_3dx - temp_uav_pos[1],goal_3dy - temp_uav_pos[2]);
  ROS_INFO("Goal Angle = %f",goal_angle);

  // consecutive_angle = atan2(temp_uav_pos[4] - previous_y, temp_uav_pos[3] - previous_x);
  consecutive_angle = kiang;
  ROS_INFO("Consecutive Angle = %f",consecutive_angle);
  
  if((previous_x == 0 && previous_y == 0)){
    u = 0;
    ROS_INFO("Angle = %f",u); 
  }
  else{
    u = 1 * ang_wrap(consecutive_angle - goal_angle);
    ROS_INFO("Angle = %f",u);
  }
      
  previous_x = temp_uav_pos[3];
  previous_y = temp_uav_pos[4];

  return(u);
}

// making sure the angle is between pi and -pi
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

// distance between 2 lat lons. the lat lons are in radian's
double distance(double lat1, double lon1, double lat2, double lon2)
{
  //ROS_INFO("lat1= %f, lon1= %f, lat2=%f, lon2%f",lat1,lon1,lat2,lon2);
  //lat1 = deg2rad(lat1);
  //lat2 = deg2rad(lat2);
  //lon1 = deg2rad(lon1);
  //lon2 = deg2rad(lon2);
  double dlon = deg2rad(lon2 - lon1);
  double dlat = deg2rad(lat2 - lat1);
  double a = sin(dlat/2)*sin(dlat/2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)*sin(dlon/2);
  a = 2*atan2( sqrt(a), sqrt(1-a) );
  double dist =  6371000*a; //637100 is the radius of earth
  //    ROS_INFO("a= %f, distance %f",a, dist);
  return(dist);
}

// bearing between 2 lat lons. the lat lons are in radian's
double bearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  lon1 = deg2rad(lon1);
  lon2 = deg2rad(lon2);
  double x = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1);
  double y = sin(lon2-lon1)*cos(lat2);
  double bearingValue = atan2(y,x);
  // ROS_INFO("bearing value %f",bearingValue);
  return(bearingValue);
}
