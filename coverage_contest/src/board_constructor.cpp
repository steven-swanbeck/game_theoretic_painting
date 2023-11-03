#include "coverage_contest/board_constructor.h"

BoardConstructor::BoardConstructor ()
{
    loadCloudData();

    float discretization {3.0}; // 3 looks pretty good for the map we have

    OctreeData map_data;
    map_data.name = "map";
    extractOctreeData(map_, discretization, map_data);

    OctreeData marked_data;
    marked_data.name = "marked";
    extractOctreeData(marked_, discretization, marked_data);

    saveOctreeDataClouds(map_data);
    saveOctreeDataClouds(marked_data);

    ros::spin();
}

void BoardConstructor::extractOctreeData (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointT> centroid_octree (octree_resolution);
    centroid_octree.defineBoundingBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

    PointCloud::Ptr input (new PointCloud);
    pcl::fromROSMsg(cloud, *input);

    centroid_octree.setInputCloud(input);
    centroid_octree.addPointsFromInputCloud();
    centroid_octree.getVoxelCentroids(output.centroids);

    pcl::octree::OctreePointCloudPointVector<PointT> octree (octree_resolution);
    octree.defineBoundingBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it) {
        pcl::Indices indexVector;
        pcl::octree::OctreeNode* node = it.getCurrentOctreeNode();
        pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
        container.getPointIndices (indexVector);

        if (indexVector.size() < 1) {continue;}

        PointCloud::Ptr cluster (new PointCloud);

        for (const auto &i : indexVector) {
            cluster->points.push_back((*input)[i]);
        }

        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*cluster, cluster_msg);
        output.clusters.push_back(cluster_msg);
    }
    std::cout << "[BoardConstructor] Extracted " << output.clusters.size() << " clusters and " << output.centroids.size() << " centroids from the provided cloud using a resolution of " << octree_resolution << ". " << std::endl;
}

PointT BoardConstructor::colorPairRandomUniform (PointCloud::Ptr &cloud, const PointT &centroid)
{
    std::mt19937 gen(rd_());
    std::uniform_int_distribution<> distr(0, 255);
    int r {distr(gen)};
    int g {distr(gen)};
    int b {distr(gen)};

    for (std::size_t i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }

    PointT colored_centroid;
    pcl::copyPoint(centroid, colored_centroid);
    colored_centroid.r = r;
    colored_centroid.g = g;
    colored_centroid.b = b;

    return colored_centroid;
}

void BoardConstructor::saveOctreeDataClouds (const OctreeData &input)
{
    PointCloud::Ptr volumes (new PointCloud);
    PointCloud::Ptr centroids (new PointCloud);

    for (std::size_t i = 0; i < input.centroids.size(); i++) {
        PointCloud::Ptr cloud (new PointCloud);
        pcl::fromROSMsg(input.clusters[i], *cloud);
        
        PointT colored_centroid {colorPairRandomUniform(cloud, input.centroids[i])};
        *volumes += *cloud;
        centroids->points.push_back(colored_centroid);
    }

    pcl::PCDWriter writer;
    writer.write("/home/steven/" + input.name + "_clusters.pcd", *volumes, false);
    centroids->height = 1;
    centroids->width = centroids->points.size();
    writer.write("/home/steven/" + input.name + "_centroids.pcd", *centroids, false);
    std::cout << "Finished coloring and saving clouds!" << std::endl;
}

void BoardConstructor::loadCloudData ()
{
    // TODO make this use params for directory set by user
    loadCloudasMsg ("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", map_);
    loadCloudasMsg ("/home/steven/game_theoretic_painting/src/models/clouds/revised/marked.pcd", marked_);
}

void BoardConstructor::loadCloudasMsg (const std::string &dir, sensor_msgs::PointCloud2 &msg)
{
    pcl::PCDReader reader;
    PointCloud::Ptr cloud (new PointCloud);
    reader.read(dir, *cloud);
    std::cout << "Loaded cloud has " << cloud->points.size() << " points." << std::endl;
    pcl::toROSMsg(*cloud, msg);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_include");
    BoardConstructor board_constructor;
    return 0;
}
