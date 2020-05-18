/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

    @brief ROS class for detecting obstacles in a point cloud.

   This class produces a point cloud containing all points that lie on
   an obstacle and are taller than the @c height_threshold parameter.

Subscribes:

- @b velodyne_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR

Publishes:

- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles
  
- @b obstacles [velodyne_height_map::obstacles] the bounding boxes for the obstacles

- @b visualization_marker_array [visualization_msgs::MarkerArray] lines defining the bounding boxes


@author David Claridge, Michael Quinlan 

*/

#include <velodyne_height_map/heightmap.h>
#include "math.h"


namespace velodyne_height_map {

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
double rotation_angle = (M_PI/12);

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  priv_nh.param("cell_size", m_per_cell_, 0.5);
  priv_nh.param("full_clouds", full_clouds_, false);
  priv_nh.param("grid_dimensions", grid_dim_, 100);
  priv_nh.param("height_threshold", height_diff_threshold_, 0.2);
  priv_nh.param("car_height", car_height_,2.0);

  mindistance=1000.0;
  ROS_INFO_STREAM("height map parameters: "
                  << grid_dim_ << "x" << grid_dim_ << ", "
                  << m_per_cell_ << "m cells, "
                  << height_diff_threshold_ << "m threshold, "
                  << (full_clouds_? "": "not ") << "publishing full clouds " << car_height_ <<"m car height. ");

  // Set up publishers
  
  obstacle_publisher_ = node.advertise<VPointCloud>("velodyne_obstacles",1);
  clear_publisher_ = node.advertise<VPointCloud>("velodyne_clear",1);
  scan_filter_ = node.advertise<VPointCloudI>("scan_filter",1);// for testing filters
  Box_pub = node.advertise<velodyne_height_map::Obstacles>("obstacles",1);
  marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  stamped_marker_pub = node.advertise<velodyne_height_map::Stamped_markers_array>("stamped_marker_array", 1);
  // subscribe to Velodyne data points
  velodyne_scan_ = node.subscribe("velodyne_filtered", 10,
                                  &HeightMap::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));
}

HeightMap::~HeightMap() {}



//functions to distinguish objects
int HeightMap::isSafe(int M[][100], int x, int y, int visited[][100])
{
    // row number is in range, column number is in range and value is 1 
    // and not yet visited
    return (x >= 0) && (x < grid_dim_) &&     
           (y >= 0) && (y < grid_dim_) &&      
           (M[x][y] && !visited[x][y]); 
}

void HeightMap::DFS(int M[][100], int x, int y, int visited[][100], ObjIdentifier *obj, float min[][100], float max[][100])
{
    // These arrays are used to get row and column numbers of 48 neighbours 
    // of a given cell
    static int rowNbr[] = {-3,-3,-3,-3,-3,-3,-3,-2,-1,0,1,2,3,3,3,3,3,3,3,2,1,0,-1,-2,-2,-2,-2,-2,-2,-1,0,1,2,2,2,2,2,1,0,-1,-1, -1, -1,  0, 0,  1, 1, 1};
    static int colNbr[] = {-3,-2,-1,0,1,2,3,3,3,3,3,3,3,2,1,0,-1,-2,-3,-3,-3,-3,-3,-3,-2,-2,0,1,2,2,2,2,2,1,0,-1,-2,-2,-2,-2,-1,  0,  1, -1, 1, -1, 0, 1};
 
    // Mark this cell as visited
    visited[x][y] = true;
    float min_h,max_h;
    min_h = min[x][y];
    max_h = max[x][y];
    
    // finding the size of the object
    (*obj).maxcol=MAX(y,(*obj).maxcol);
    (*obj).mincol=MIN(y,(*obj).mincol);
    (*obj).maxrow=MAX(x,(*obj).maxrow);
    (*obj).minrow=MIN(x,(*obj).minrow);
    (*obj).maxz=MAX(max_h,(*obj).maxz);
    (*obj).minz=MIN(min_h,(*obj).minz);

    // Recur for all connected neighbours
    for (int k = 0; k < 48; ++k)
        if (isSafe(M, x + rowNbr[k], y + colNbr[k], visited) )
            DFS(M, x+ rowNbr[k], y + colNbr[k], visited, &(*obj),min,max);
}

void HeightMap::ClassifyClusters(int M[][100],std::vector<ObjIdentifier> *list, float min[][100], float max[][100])
{
    // Make a bool array to mark visited cells.
    // Initially all cells are unvisited
    // ROS_INFO_STREAM("\n here1" << "\n");
    
    int visited[100][100];
     memset(visited, 0, sizeof(visited));
    ObjIdentifier objidentify;
    //std::vector<ObjIdentifier> list;
    int count=0;
    //ROS_INFO_STREAM(count << "\n"); 
    int maxrow,minrow,maxcol,mincol;
  float maxz, minz;
   for (int i = 0; i < grid_dim_; ++i)
        for (int j = 0; j < grid_dim_; ++j)
            if (M[i][j] && !visited[i][j]) // If a cell with value 1 is not
            {                         // visited yet, then new obstacle found   
    //count++;
    //ROS_INFO_STREAM(count << "\n"); //displays no. of objects found 
    minrow=grid_dim_+1;
    maxrow=-1;
    mincol=grid_dim_+1;
    maxcol=-1;
    objidentify.maxcol=maxcol;
    objidentify.mincol=mincol;
    objidentify.maxrow=maxrow;
    objidentify.minrow=minrow;
    objidentify.maxz=maxz;
    objidentify.minz=minz;
    objidentify.type=0;
    DFS(M, i, j, visited, &objidentify, min, max);     // Visit all cells in this obstacle.
    (*list).push_back(objidentify);                   //label the object
            }
    //return list;
    //return count;
}

void HeightMap::IdentifyClusters(std::vector <ObjIdentifier> *list)
{
   int i,xmin,xmax,ymin,ymax,width,length;
   for (i=0;i<(*list).size();i++)
   {
  xmin=(*list)[i].minrow;
  xmax=(*list)[i].maxrow;
  ymin=(*list)[i].mincol;
  ymax=(*list)[i].maxcol;
  width=ymax-ymin;  // width of the object in metres/cell size
  length=xmax-xmin; //length of the object in metres/cll size
  //type 1 - car
  //type 2 - larger vehicle such as truck etc.
  //type 3 - autorickshaw, rickshaw etc.
  //type 4 - bike/cycle/scooter etc.
  //type 5 - others such as trees , walls etc.
  //type 6 - people
  //type 7 - cant decipher

// numbers in if else blocks are in metres . Change them as per need to distinguish obstacles
  if ( (double)width>=1.25/m_per_cell_ && (double)width <=2.1/m_per_cell_ && (double)length>=3.75/m_per_cell_ && (double)length<=5/m_per_cell_ ) 
    (*list)[i].type=1;
  else if ((double)width>=2.25/m_per_cell_ && (double)width <=3/m_per_cell_ && (double)length>=8.75/m_per_cell_ && (double)length<=10/m_per_cell_ )
    (*list)[i].type=2;
  else if ((double)width>=0.95/m_per_cell_ && (double)width <=1.5/m_per_cell_ && (double)length>=2.5/m_per_cell_ && (double)length<=3.75/m_per_cell_ )
    (*list)[i].type=3;
  else if ((double)width>=0.5/m_per_cell_ && (double)width <=1.25/m_per_cell_ && (double)length>=1.75/m_per_cell_ && (double)length<=2.5/m_per_cell_ )
    (*list)[i].type=4;
  else if ((double)width>=0.5/m_per_cell_ && (double)width <=1/m_per_cell_ && (double)length>=0.5/m_per_cell_ && (double)length<=1/m_per_cell_ )
    (*list)[i].type=6;
  else if ((double)width>=0.2/m_per_cell_ && (double)length>=1.25/m_per_cell_ )
    (*list)[i].type=5;
  else (*list)[i].type=7;
   }
} 
// end of functions to distinguish objects




void HeightMap::constructFullClouds(const VPointCloudI::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  memset(&init, 0, grid_dim_*grid_dim_);
  
  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }
  
//code snippet to find closest distance object from the car
  int here=1;
  double dist,mindist;
  for (unsigned i = 0; i < npoints; ++i) {
    double x = scan->points[i].x;
    double y = scan->points[i].y;
    double z = scan->points[i].z;
    dist=sqrt(x*x + y*y);
    
    if (here==1 && z<=-0.4) {mindist=dist;here=0;}
    else if ( z<=-0.4) mindist=MIN(mindist,dist);
  }
 mindistance = MIN(mindistance,mindist);
 ROS_INFO_STREAM("min dist is " << mindistance);

// ignoring the objects above the car
 for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) { 
  if (min[x][y]>=car_height_){
  max[x][y]=min[x][y];
  //ROS_INFO_STREAM("min[x][y] is " << min[x][y] <<"\n");
  }
      }
  }

  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {   
        obstacle_cloud_.points[obs_count].x = scan->points[i].x;
        obstacle_cloud_.points[obs_count].y = scan->points[i].y;
        obstacle_cloud_.points[obs_count].z = scan->points[i].z;
        obstacle_cloud_.points[obs_count].label=scan->points[i].intensity;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) scan->points[i].intensity;
        obs_count++;
      } else {
        clear_cloud_.points[empty_count].x = scan->points[i].x;
        clear_cloud_.points[empty_count].y = scan->points[i].y;
        clear_cloud_.points[empty_count].z = scan->points[i].z;
  clear_cloud_.points[obs_count].label=scan->points[i].intensity;
        //clear_cloud_.channels[0].values[empty_count] = (float) scan->points[i].intensity;
        empty_count++;
      }
    }
  }
}

void HeightMap::constructGridClouds(const VPointCloudI::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[100][100];
  float max[100][100];
  //float num_obs[grid_dim_][grid_dim_];
  //float num_clear[grid_dim_][grid_dim_];
  int num_obs[100][100], num_clear[100][100];
  //int num_obs[grid_dim_][grid_dim_];
  //int num_clear[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  std::vector<ObjIdentifier> list;
  velodyne_height_map::Obstacle obstacle;
  //list.resize(1000);
  //memset(&init, 0, grid_dim_*grid_dim_);
  
  
  //lines
  visualization_msgs::Marker lines;
  lines.header.frame_id = "velodyne";
  lines.header.stamp = ros::Time::now();
  lines.type = visualization_msgs::Marker::LINE_LIST;
  //lines.action = visualization_msgs::Marker::DELETE;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.05;
  lines.scale.y = 0.05;
  lines.color.a = 1.0;
  lines.ns = "my lines";
  lines.id = 1;
  lines.lifetime = ros::Duration(0.01); 
  lines.points.clear();
  lines.color.r = 0.0f ;
  lines.color.g = 0.0f;
  lines.color.b = 1.0f ;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y]=false;
      num_obs[x][y]=0;
      num_clear[x][y]=0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        num_obs[x][y] = 0;
        num_clear[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }

// ignoring the objects above the car
 for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) { 
  if (min[x][y]>=car_height_){
  max[x][y]=min[x][y];
  //ROS_INFO_STREAM("min[x][y] is " << min[x][y] <<"\n");
  }
     }
  }

  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
        num_obs[x][y]++;
      } else {
        num_clear[x][y]++;
      }
    }
  }
  
  //ROS_INFO_STREAM("here" << "\n");
  ClassifyClusters(num_obs,&list, min, max);
  IdentifyClusters(&list);
  
  

  for (int i=0;i<list.size();i++)
  {
  ROS_INFO_STREAM(list[i].type);
  }
  ROS_INFO_STREAM("\n");
  // create clouds from grid
  double grid_offset=grid_dim_/2.0*m_per_cell_;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      /*if (num_obs[x][y]>0) {

        obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obs_count++;
      }*/
      if (num_clear[x][y]>0) {
        clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].z = height_diff_threshold_;
        //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
        empty_count++;
      }
    }
  }
geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
  for (int i=0;i<list.size();i++)
  {
  for (int j=list[i].minrow;j<=list[i].maxrow;j++)
  {
    for (int k=list[i].mincol;k<=list[i].maxcol;k++)
    {
      if (num_obs[j][k]>0){
      obstacle_cloud_.points[obs_count].x = -grid_offset + (j*m_per_cell_+m_per_cell_/2.0);
            obstacle_cloud_.points[obs_count].y = -grid_offset + (k*m_per_cell_+m_per_cell_/2.0);
            obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
            if (list[i].type==1) 
      { obstacle_cloud_.points[obs_count].r=255;    // white -car
        obstacle_cloud_.points[obs_count].g=255;
        obstacle_cloud_.points[obs_count].b=255;
      }
      else if (list[i].type==2) 
      { obstacle_cloud_.points[obs_count].r=0;    // blue  -truck
        obstacle_cloud_.points[obs_count].g=0;
        obstacle_cloud_.points[obs_count].b=255;
      }
      else if (list[i].type==3) 
      { obstacle_cloud_.points[obs_count].r=255;  //red -rickshaw
        obstacle_cloud_.points[obs_count].g=0;
        obstacle_cloud_.points[obs_count].b=0;
      }
      else if (list[i].type==4) 
      { obstacle_cloud_.points[obs_count].r=0;    //green - bike
        obstacle_cloud_.points[obs_count].g=255;
        obstacle_cloud_.points[obs_count].b=0;
      }
      else if (list[i].type==5) 
      { obstacle_cloud_.points[obs_count].r=255;  //yellow - tree,wall etc
        obstacle_cloud_.points[obs_count].g=255;
        obstacle_cloud_.points[obs_count].b=0;
      }
      else if (list[i].type==6) 
      { obstacle_cloud_.points[obs_count].r=0;    //cyan - people
        obstacle_cloud_.points[obs_count].g=255;
        obstacle_cloud_.points[obs_count].b=255;
      }
      else if (list[i].type==7) 
      { obstacle_cloud_.points[obs_count].r=255;  //pink - cant decipher
        obstacle_cloud_.points[obs_count].g=0;
        obstacle_cloud_.points[obs_count].b=255;
      }
            obs_count++;
      }
    }
  }
  //Code for the bounding box here
  //if(list[i].type!=5 && list[i].type!=7){
  float x1,x2,y1,y2,z1,z2;
  x1= -grid_offset + (list[i].minrow*m_per_cell_+m_per_cell_/2.0);
  x2= -grid_offset + (list[i].maxrow*m_per_cell_+m_per_cell_/2.0);
  y1= -grid_offset + (list[i].mincol*m_per_cell_+m_per_cell_/2.0);
  y2= -grid_offset + (list[i].maxcol*m_per_cell_+m_per_cell_/2.0);
  z1= list[i].minz;
  z2= list[i].maxz;
  
  //Publishes the diagonal points for the 3D bounding box
  if(x1!=x2 && y1!=y2){
  obstacle.min_x = x1;
  obstacle.max_x = x2;
  obstacle.min_y = y1;
  obstacle.max_y = y2;
  obstacle.min_z = z1;
  obstacle.max_z = z2;
  obstacle.obstacle_class = list[i].type;
  obstacle.obstacleID = i;
  obstacles_array.obstacles.push_back(obstacle);
  
  //This code is for visualization of bounding boxes on rviz
  //rviz does not support wireframe cube so 12 lines are needed to be plotted for each of box
  //Note that this slows down rviz if the number of boxes is high
  p1.x = x1; p1.y = y1; p1.z = z1;
  p2.x = x2; p2.y = y1; p2.z = z1;
  p3.x = x1; p3.y = y1; p3.z = z2;
  p4.x = x2; p4.y = y1; p4.z = z2;
  p5.x = x1; p5.y = y2; p5.z = z1;
  p6.x = x2; p6.y = y2; p6.z = z1;
  p7.x = x1; p7.y = y2; p7.z = z2;
  p8.x = x2; p8.y = y2; p8.z = z2;
  lines.points.push_back(p1); lines.points.push_back(p2);
  lines.points.push_back(p1); lines.points.push_back(p3);
  lines.points.push_back(p2); lines.points.push_back(p4);
  lines.points.push_back(p3); lines.points.push_back(p4);
  lines.points.push_back(p5); lines.points.push_back(p6);
  lines.points.push_back(p5); lines.points.push_back(p7);
  lines.points.push_back(p6); lines.points.push_back(p8);
  lines.points.push_back(p7); lines.points.push_back(p8);
  lines.points.push_back(p1); lines.points.push_back(p5);
  lines.points.push_back(p2); lines.points.push_back(p6);
  lines.points.push_back(p3); lines.points.push_back(p7);
  lines.points.push_back(p4); lines.points.push_back(p8);
  line_list.markers.push_back(lines);}//}
  }
  obstacles_array.numberOfDetections = list.size();
  list.clear();
  
}

/** point cloud input callback */
void HeightMap::processData(const VPointCloudI::ConstPtr &scan)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_new(new pcl::PointCloud<pcl::PointXYZI>);
  //scan_new->points.size() = scan->points.size();
  //scan_new = scan;
  scan_new->header.stamp = scan->header.stamp;
  scan_new->header.frame_id = scan->header.frame_id;
  size_t npoints = scan->points.size();
  scan_new->points.resize(npoints);

  for (unsigned i = 0; i < npoints; ++i) {
    scan_new->points[i].y = scan->points[i].y;
    scan_new->points[i].z = (scan->points[i].z)*cos(rotation_angle) - (scan->points[i].x)*sin(rotation_angle);
    scan_new->points[i].x = (scan->points[i].z)*sin(rotation_angle) + (scan->points[i].x)*cos(rotation_angle);
    //ROS_INFO("%f %f", scan->points[i].y, scan_new->points[i].y);
  }
  // if ((obstacle_publisher_.getNumSubscribers() == 0)
  //     && (clear_publisher_.getNumSubscribers() == 0)
  //     && (scan_filter_.getNumSubscribers() == 0))
  //   return;
  
  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = scan->header.stamp;
  obstacle_cloud_.header.frame_id = scan->header.frame_id;

  // pass along original time stamp and frame ID
  clear_cloud_.header.stamp = scan->header.stamp;
  clear_cloud_.header.frame_id = scan->header.frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  obstacle_cloud_.points.resize(npoints);
  //obstacle_cloud_.channels[0].values.resize(npoints);

  clear_cloud_.points.resize(npoints);
  //clear_cloud_.channels[0].values.resize(npoints);
  pcl::CropBox<VPointI> Ptr;
  Ptr.setInputCloud (scan_new);
  minpt << -7.5,-7.5,-100,0;
  maxpt << 7.5,7.5,100,0;
  Ptr.setMin (minpt);
  Ptr.setMax (maxpt);
  Ptr.filter (scan_filter);
  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  if (full_clouds_)
    constructFullClouds(scan_new,npoints,obs_count, empty_count);
  else
    constructGridClouds(scan_new,npoints,obs_count, empty_count);
  
  obstacle_cloud_.points.resize(obs_count);
  //obstacle_cloud_.channels[0].values.resize(obs_count);

  clear_cloud_.points.resize(empty_count);
  //clear_cloud_.channels[0].values.resize(empty_count);
  
  // if (scan_filter_.getNumSubscribers() > 0)
    scan_filter_.publish(scan_filter);  

  // if (obstacle_publisher_.getNumSubscribers() > 0)
    obstacle_publisher_.publish(obstacle_cloud_);

  // if (clear_publisher_.getNumSubscribers() > 0)
    clear_publisher_.publish(clear_cloud_);
   
  // if(Box_pub.getNumSubscribers()>0)
   Box_pub.publish(obstacles_array);
    
  // if(marker_pub.getNumSubscribers()>0)  

   //create a stamped visual
   stamped_marker_array.message = line_list;
   stamped_marker_array.header.stamp =  pcl_conversions::fromPCL(scan_new->header.stamp);
   stamped_marker_array.header.frame_id = "velodyne";
   stamped_marker_pub.publish(stamped_marker_array);
   marker_pub.publish (line_list);

    line_list.markers.clear();
     
}

} // namespace velodyne_height_map
