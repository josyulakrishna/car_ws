/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */


#ifndef _HEIGHT_MAP_H_
#define _HEIGHT_MAP_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

#include <velodyne_height_map/Obstacle.h>
#include <velodyne_height_map/Obstacles.h>
#include <velodyne_height_map/Stamped_markers_array.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_height_map {

// shorter names for point cloud types in this namespace
typedef pcl::PointXYZRGBL VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZI VPointI;
typedef pcl::PointCloud<VPointI> VPointCloudI;

class ObjIdentifier
{
public:
int maxcol,mincol,maxrow,minrow,type;
float maxz, minz;
};

class HeightMap
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  HeightMap(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~HeightMap();

  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void processData(const VPointCloudI::ConstPtr &scan);

private:
  void constructFullClouds(const VPointCloudI::ConstPtr &scan, unsigned npoints,
                           size_t &obs_count, size_t &empty_count);
  void constructGridClouds(const VPointCloudI::ConstPtr &scan, unsigned npoints,
                           size_t &obs_count, size_t &empty_count);

  
  int isSafe(int M[][100], int x, int y, int visited[][100]);
  void DFS(int M[][100], int x, int y, int visited[][100],ObjIdentifier *obj, float min[][100], float maxz[][100]);
  void ClassifyClusters(int M[][100],std::vector<ObjIdentifier> *list, float min[][100], float maxz[][100]);
  void IdentifyClusters(std::vector <ObjIdentifier> *list);
  // Parameters that define the grids and the height threshold
  // Can be set via the parameter server
  int grid_dim_;
  double m_per_cell_;
  double height_diff_threshold_;
  bool full_clouds_;
  double car_height_;
  double mindistance;
  visualization_msgs::MarkerArray line_list;
  velodyne_height_map::Obstacles obstacles_array;
  velodyne_height_map::Stamped_markers_array stamped_marker_array;
  
  // Point clouds generated in processData
  VPointCloud obstacle_cloud_;            
  VPointCloud clear_cloud_;            
  
  // ROS topics
  ros::Subscriber velodyne_scan_;
  ros::Publisher obstacle_publisher_;
  ros::Publisher clear_publisher_;
  ros::Publisher Box_pub;
  ros::Publisher marker_pub;
  ros::Publisher stamped_marker_pub;
  
  VPointCloudI scan_filter;
  ros::Publisher scan_filter_;
  double radius;
  int numpts;
  Eigen::Vector4f minpt;
  Eigen::Vector4f maxpt;
  
};

} // namespace velodyne_height_map

#endif
