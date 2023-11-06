#ifndef CLUSTERING_SERVER_H
#define CLUSTERING_SERVER_H

#include "point.h"
#include "common.h"
#include <thread>
#include "haphephobia/cluster_pc.h"

using namespace std::chrono_literals;

class ClusteringServer
{
public:
    ClusteringServer();
private:
    bool createClusters(haphephobia::cluster_pc::Request &req, haphephobia::cluster_pc::Response &res);

    bool negative_exclude_bool_;
    float cluster_confidence_threshold_;
    int min_cluster_size_;
    int max_cluster_size_;
    int num_neighbors_;
    float smoothness_threshold_;
    float curvature_threshold_;
    std::string output_frame_;

    ros::NodeHandle nh_;
    ros::ServiceServer clustering_serivce_;
};

#endif //CLUSTERING_SERVER_H
