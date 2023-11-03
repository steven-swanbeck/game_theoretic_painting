#ifndef BOARD_CONSTRUCTOR_H
#define BOARD_CONSTRUCTOR_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include "board.h"
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <random>

class BoardConstructor
{
public:
    BoardConstructor();
private:
    void loadCloudData ();
    void loadCloudasMsg (const std::string &dir, sensor_msgs::PointCloud2 &msg);

    void extractOctreeData (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output);
    PointT colorPairRandomUniform (PointCloud::Ptr &cloud, const PointT &centroid);
    void saveOctreeDataClouds (const OctreeData &input);

    ros::NodeHandle nh_;
    
    sensor_msgs::PointCloud2 map_;
    sensor_msgs::PointCloud2 marked_;

    std::random_device rd_;
};

#endif // BOARD_CONSTRUCTOR_H
