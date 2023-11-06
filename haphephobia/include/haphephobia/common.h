#ifndef HAPHEPHOBIA_COMMON_H
#define HAPHEPHOBIA_COMMON_H

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/region_growing.h>

#endif // HAPHEPHOBIA_COMMON_H
