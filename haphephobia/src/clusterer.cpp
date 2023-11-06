// Includes
#include "haphephobia/clusterer.h"
#include "haphephobia/cluster_pc.h"
// #include "surface_repair/process_pc.h"



// - Constructor
ClusteringServer::ClusteringServer()
{
    if(!nh_.param<bool>("/surface_repair/processing/clustering/exclude_negatives", negative_exclude_bool_, true));
    // if( !nh_.param<float>("/surface_repair/processing/clustering/confidence_threshold", cluster_confidence_threshold_, 0.5));
    if(!nh_.param<int>("/surface_repair/processing/clustering/min_cluster_size", min_cluster_size_, 50));
    if(!nh_.param<int>("/surface_repair/processing/clustering/max_cluster_size", max_cluster_size_, 1000000));
    if(!nh_.param<int>("/surface_repair/processing/clustering/num_neighbors", num_neighbors_, 10));
    if(!nh_.param<float>("/surface_repair/processing/clustering/smoothness_threshold", smoothness_threshold_, 30.0));
    if(!nh_.param<float>("/surface_repair/processing/clustering/curvature_threshold", curvature_threshold_, 5.0));
    nh_.param<std::string>("/surface_repair/general/output_frame", output_frame_, "map");

    clustering_serivce_ = nh_.advertiseService("cluster_process", &ClusteringServer::createClusters, this);

    ros::spin();
}



// - Primary callback
bool ClusteringServer::createClusters (haphephobia::cluster_pc::Request &req, haphephobia::cluster_pc::Response &res) 
{
    PointCloud::Ptr input (new PointCloud);

    // - checking to see if we only want to include positive returns and reading in cloud
    if(!nh_.param<int>("/surface_repair/processing/clustering/min_cluster_size", min_cluster_size_, 50));
    if(!nh_.param<int>("/surface_repair/processing/clustering/max_cluster_size", max_cluster_size_, 1000000));
    if(!nh_.param<int>("/surface_repair/processing/clustering/num_neighbors", num_neighbors_, 10));
    if(!nh_.param<float>("/surface_repair/processing/clustering/smoothness_threshold", smoothness_threshold_, 30.0));
    if(!nh_.param<float>("/surface_repair/processing/clustering/curvature_threshold", curvature_threshold_, 5.0));

    pcl::fromROSMsg(req.input, *input);

    bool success_flag {false};

    // - handling edge case when there is no data that makes it to this stage
    if (input->points.size() > 0) {

        // - defining other necessary structures
        pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        pcl::IndicesPtr indices (new std::vector <int>);

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud (*input, *normals);

        // - rgs
        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size_);
        reg.setMaxClusterSize(max_cluster_size_);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(num_neighbors_);
        reg.setInputCloud(input);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold_ / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold_);

        std::vector <pcl::PointIndices> clusters;
        reg.extract(clusters);

        if (clusters.size() > 0) {
            // - composing clusters into set of pc2s to fill in data
            sensor_msgs::PointCloud2 cluster_clouds[clusters.size()];
            for (size_t i = 0; i < clusters.size(); i++) {
                PointCloud::Ptr temp_cloud (new PointCloud);
                sensor_msgs::PointCloud2 temp_msg;

                for (size_t j = 0; j < clusters[i].indices.size(); j++) {
                    temp_cloud->push_back((*input)[clusters[i].indices[j]]);
                }
                // - now convert to pc2 message and add to collection
                pcl::toROSMsg(*temp_cloud, temp_msg);
                temp_msg.header.frame_id = output_frame_;
                res.clusters.push_back(temp_msg);
            }

            // - adding colored cloud
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
            sensor_msgs::PointCloud2 colored_msg;
            pcl::toROSMsg(*colored_cloud, colored_msg);
            colored_msg.header.frame_id = output_frame_;
            res.colored = colored_msg;
            success_flag = true;
        }
    }

    if (success_flag == false) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2 colored_msg;
        pcl::toROSMsg(*colored_cloud, colored_msg);
        colored_msg.header.frame_id = output_frame_;
        res.colored = colored_msg;
    }

    return true;
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "clustering_server");
    ClusteringServer clustering_server;
    return 0;
}
