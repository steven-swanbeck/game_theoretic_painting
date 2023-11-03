#include "haphephobia/common.h"
#include "haphephobia/point.h"

struct Board
{
    std::string name;
    sensor_msgs::PointCloud2 cloud_msg;
    std::vector<sensor_msgs::PointCloud2> clusters;

};

struct OctreeData
{
    std::string name;
    std::vector<sensor_msgs::PointCloud2> clusters;
    std::vector<PointT, Eigen::aligned_allocator<PointT>> centroids;
};
