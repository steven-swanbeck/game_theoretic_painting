#ifndef VIRTUAL_FIXTURE_GENERATION_SERVER
#define VIRTUAL_FIXTURE_GENERATION_SERVER

#include "common.h"
#include "point.h"
#include "haphephobia/create_vfs.h"
#include <pcl/surface/mls.h>

class VFGenerationServer
{
public:
    VFGenerationServer();
private:
    bool generationCallback (haphephobia::create_vfs::Request &req, haphephobia::create_vfs::Response &res);
    void splitClusters (const std::vector<sensor_msgs::PointCloud2> &raw_clusters, std::vector<sensor_msgs::PointCloud2> &split_clusters);
    void upsampleCloud (PointCloud::Ptr &input, PointCloud::Ptr &output, double sampling_radius=0.005, double step_size=0.005, double search_radius=0.03, int polynomial_order=2);
    void downsampleCloud (PointCloud::Ptr &input, PointCloud::Ptr &reference, PointCloud::Ptr &output, float voxel_leaf_size);

    ros::NodeHandle nh_;
    ros::ServiceServer vf_generation_service_;

    float surface_offset_;
    float voxel_leaf_size_;
    std::string output_frame_;

    float base_relative_offset_;
    float base_bias_offset_;
    bool ground_base_;
    float base_z_offset_;
    float fixed_base_height_;
    std::vector<float> reference_axis_;
};

#endif //VIRTUAL_FIXTURE_GENERATION_SERVER
