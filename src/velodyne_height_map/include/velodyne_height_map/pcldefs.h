#pragma once

#include <limits>
using namespace std;

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#define ALGORITHM_POINT_TO_POINT  "p2p"
#define ALGORITHM_POINT_TO_PLANE  "p2pl"


typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud <pcl::PointXYZ>::Ptr PointCloudPtr;

typedef pcl::PointCloud <pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud <pcl::PointXYZRGB>::Ptr PointCloudRGBPtr;

typedef pcl::PointCloud <pcl::PointNormal> PointCloudNormal;
typedef pcl::PointCloud <pcl::PointNormal>::Ptr PointCloudNormalPtr;

typedef pcl::PointCloud <pcl::PointXYZI> PointCloudI;
typedef pcl::PointCloud <pcl::PointXYZI>::Ptr PointCloudIPtr;

typedef pcl::PointCloud <pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr PointCloudRGBNormalPtr;

typedef pcl::PointCloud <pcl::PointXY> PointCloudxy;
typedef pcl::PointCloud <pcl::PointXY>::Ptr PointCloudxyPtr;


// using namespace pcl::visualization;
// typedef boost::shared_ptr <PCLVisualizer> PCLVisualizerPtr;

template <typename CloudType>
void computeCentroid (CloudType cloud, pcl::PointXYZ& centroid)
{
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;

  int N  = cloud->points.size ();

  for (int i = 0; i < N; i++) {
    centroid.x += cloud->points[i].x;
    centroid.y += cloud->points[i].y;
    centroid.z += cloud->points[i].z;
  }

  centroid.x = centroid.x / N;
  centroid.y = centroid.y / N;
  centroid.z = centroid.z / N;
}

template <typename CloudType, typename CloudTypePtr>
CloudTypePtr downsamplePointCloud (CloudTypePtr cloud, float percentage)
{
  if (percentage == 100.0) {
    return cloud;
  }
  float downsample_ratio = percentage / 100.0;

  int N1 = cloud->points.size ();
  int N2 = N1 * downsample_ratio;

  CloudTypePtr downsampled_cloud (new CloudType);

  for (int i = 0; i < N2; i++) {
    int k = rand () % N1;
    downsampled_cloud->points.push_back (cloud->points[k]);
  }

  downsampled_cloud->width = 1;
  downsampled_cloud->height = downsampled_cloud->points.size ();

  //cout << "# of points in the downsampled cloud = " 
  //     << downsampled_cloud->height  << endl;

  return downsampled_cloud;
}

template <typename CloudTypePtr, typename PointType>
float findRMS (CloudTypePtr& source, CloudTypePtr& target, 
               int& N,
               float threshold = numeric_limits <float>::max ())
{
  if (target->points.size () == 0) {
    return numeric_limits <float>::max ();
  }
  pcl::KdTreeFLANN <PointType> kdtree;
  kdtree.setInputCloud (target);

  int K = 1;

  vector <int> ids (K);
  vector <float> dists (K);

  float rms = 0.0;
  N = 0;

  for (int i = 0; i < source->points.size (); i++) {
    if (kdtree.nearestKSearch (source->points[i], K, ids, dists) > 0) {
      if (dists[0] < threshold) {
        N++;
        rms += dists[0];
      }
    }
  }

  if (N == 0) {
    rms = numeric_limits <float>::max ();
    return rms;
  }

  rms /= N;
  return rms;
}

template <typename CloudType, typename CloudTypePtr, typename PointType>
CloudTypePtr filterPointCloud (CloudTypePtr cloud, string field, 
                                                  float low, float high)
{
  CloudTypePtr cloud_filtered (new CloudType);
  pcl::PassThrough <PointType> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (field);
  pass.setFilterLimits (low, high);
  pass.filter (*cloud_filtered);
  return cloud_filtered;
}

template <typename CloudType>
void addPoint (CloudType cloud, pcl::PointXYZ point)
{
  int N  = cloud->points.size ();
  for (int i = 0; i < N; i++) {
    cloud->points[i].x += point.x;
    cloud->points[i].y += point.y;
    cloud->points[i].z += point.z;
  }
}

template <typename CloudType>
void subtractPoint (CloudType cloud, pcl::PointXYZ point)
{
  int N  = cloud->points.size ();
  for (int i = 0; i < N; i++) {
    cloud->points[i].x -= point.x;
    cloud->points[i].y -= point.y;
    cloud->points[i].z -= point.z;
  }
}

template <typename PointType, typename CloudType>
void estimateNormals (CloudType cloud)
{
  pcl::NormalEstimation <PointType, PointType> normal_estimation;
  normal_estimation.setSearchMethod (typename pcl::search::KdTree <PointType>::Ptr (new pcl::search::KdTree <PointType>));
  normal_estimation.setKSearch (10);
  //normal_estimation.setRadiusSearch (1);
  normal_estimation.setInputCloud (cloud);
  normal_estimation.compute (*cloud);
  //cout << "Comoputed normals ..\n";
}

template <class CloudType>
void removeNaNsAndZerosFromPointCloud (CloudType& cloud)
{
  CloudType copy;
  pcl::copyPointCloud (cloud, copy);
  cloud.points.clear ();

  for (int i = 0; i < copy.points.size (); i++) {

    // remove zeros
    if (copy.points[i].x == 0 and copy.points[i].y == 0 and copy.points[i].z == 0) {
      continue;
    }

    // remove NaNs
    if (!pcl_isfinite (copy.points[i].x) or !pcl_isfinite (copy.points[i].y) 
        or !pcl_isfinite (copy.points[i].z)) {
      continue;
    }

    cloud.points.push_back (copy.points[i]);
  }
  cloud.width = 1;
  cloud.height = cloud.points.size ();
}


template <class CloudType>
void removeNaNsAndZerosFromPointCloudAndDecimate (CloudType& cloud, int decimateFactor)
{
  CloudType copy;
  pcl::copyPointCloud (cloud, copy);
  cloud.points.clear ();

  for (int i = 0; i < copy.points.size (); i++) {
    
    // remove zeros
    if (copy.points[i].x == 0 and copy.points[i].y == 0 and copy.points[i].z == 0) {
      continue;
    }

    // remove NaNs
    if (!pcl_isfinite (copy.points[i].x) or !pcl_isfinite (copy.points[i].y) 
        or !pcl_isfinite (copy.points[i].z)) {
      continue;
    }

    if(i % decimateFactor == 0)
    {
     cloud.points.push_back (copy.points[i]);
    }
  }
  cloud.width = 1;
  cloud.height = cloud.points.size ();
}



//template <typename PointType, typename PointTypeNormal, typename InCloudType, typename OutCloudType>
//void estimateNormals (InCloudType cloud, OutCloudType cloud_normals)
//{
//  pcl::NormalEstimation <PointType, PointTypeNormal> normal_estimation;
//  normal_estimation.setSearchMethod (typename pcl::search::KdTree <PointType>::Ptr (new pcl::search::KdTree <PointType>));
//  normal_estimation.setKSearch (10);
//  //normal_estimation.setRadiusSearch (1);
//  normal_estimation.setInputCloud (cloud);
//  normal_estimation.compute (*cloud_normals);
//  //cout << "Comoputed normals ..\n";
//}
//
