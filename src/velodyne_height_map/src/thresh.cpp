#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/pcl_base.h"

using namespace std;

float thresh;
ros::Publisher points_pub;

void passpoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (msg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-5.0, thresh);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  points_pub.publish(cloud_filtered);

}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "thresh");
  ros::NodeHandle n;

  if (argc < 1)
  {
    cout << "Enter the following arguments ..\n"
            "1) threshold height (2)\n"
            "\n";
        return -1;
  }

  thresh = atof(argv[1]);

  ros::Subscriber obstacle_sub = n.subscribe("/velodyne_points",1,passpoints);
  points_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_filtered",1);

  ros::spin();


  return (0);
}