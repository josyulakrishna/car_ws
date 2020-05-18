#Velodyne Height Map

##Command to run the node
rosrun velodyne_height_map heightmap_node

##Subscribes
- velodyne_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR
  
##Publishes
- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles
  
- @b obstacles [velodyne_height_map::obstacles] the bounding boxes for the obstacles

- @b visualization_marker_array [visualization_msgs::MarkerArray] lines defining the bounding boxes

###Structure of message for publishing obstacles - 
int32 numberOfDetections
Obstacle[] obstacles - Array of the detected obstacles


###Structure of the message Obstacle
uint8 obstacleID
uint8 obstacle_class
float64 min_x
float64 max_x
float64 min_y
float64 max_y
float64 min_z
float64 max_z


