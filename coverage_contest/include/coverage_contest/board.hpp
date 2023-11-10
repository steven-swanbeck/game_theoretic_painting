#ifndef GAME_BOARD_H
#define GAME_BOARD_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <random>
#include <math.h>
#include "agents.hpp"

// . Data Structures
struct OctreeData
{
    std::string name;
    float discretization;
    std::vector<sensor_msgs::PointCloud2> clusters;
    std::vector<PointT, Eigen::aligned_allocator<PointT>> centroids;
};

// . Helpful spatial relations
struct SpatialNode {
    int x_pos {-1};
    float x_pos_d {0};
    int x_neg {-1};
    float x_neg_d {0};
    int y_pos {-1};
    float y_pos_d {0};
    int y_neg {-1};
    float y_neg_d {0};
    int z_pos {-1};
    float z_pos_d {0};
    int z_neg {-1};
    float z_neg_d {0};
};
using SpatialGraph = std::map<int, SpatialNode>;

// . Movement game board
struct Space {
    int id;
    SpatialNode neighbors;
    std::vector<int> gantry_edges;
    std::vector<int> quadruped_edges;
    std::vector<int> drone_edges;
    PointT centroid;
    sensor_msgs::PointCloud2 cloud;
    bool is_ground_level {false};
    std::vector<int> repair_edges;
};
using MoveBoard = std::map<int, Space>;

// . Repair action board
struct RepairVolume {
    int id;
    PointT centroid;
    sensor_msgs::PointCloud2 cloud;
    bool covered {false};
};
using RepairBoard = std::map<int, RepairVolume>;

// . Overall board struct
struct Board {
    MoveBoard movement_spaces;
    RepairBoard repair_spaces;
};

// . Possible turns struct
struct Action {
    int move_id {-1};
    int repair_id {-1};
};
using PossibleMoves = std::vector<Action>;
using PossibleTurn = std::queue<Action>;
using PossibleTurns = std::vector<PossibleTurn>;

namespace board_utils
{

// . Function Prototypes
void extractOctreeData (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output);
void assignEdges (const SpatialGraph &spatial_graph, Space &node);
PointT colorPairRandomUniform (PointCloud::Ptr &cloud, const PointT &centroid);
void saveOctreeDataClouds (const OctreeData &input, const std::string &dir);
void loadCloudasMsg (const std::string &dir, sensor_msgs::PointCloud2 &msg);
SpatialNode findNeighbors (const OctreeData &data, const int &index, const double &tolerance=0.174533);
SpatialGraph generateSpatialGraph (const OctreeData &data);
void assignEdges (SpatialGraph &spatial_graph, Space &node);
void assignGantryEdges (SpatialGraph &spatial_graph, Space &node);
void assignQuadrupedEdges (SpatialGraph &spatial_graph, Space &node);
void assignDroneEdges (SpatialGraph &spatial_graph, Space &node);
int assignRepairEdge (const PointT &centroid, const std::vector<PointT, Eigen::aligned_allocator<PointT>> &centroids);
MoveBoard generateMoveBoard (const OctreeData &data, SpatialGraph &spatial_graph);
MoveBoard generateMoveBoard (const OctreeData &data);
MoveBoard generateMoveBoard (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution);
Board generateBoard (const OctreeData &move_data, SpatialGraph &spatial_graph, const OctreeData &repair_data);
Board generateBoard (const OctreeData &move_data, const OctreeData &repair_data);
Board generateBoard (const sensor_msgs::PointCloud2 &move_cloud, const float &move_octree_resolution, const sensor_msgs::PointCloud2 &repair_cloud, const float &repair_octree_resolution);

// . Function Definitions
Board generateBoard (const OctreeData &move_data, SpatialGraph &spatial_graph, const OctreeData &repair_data)
{
    Board board;

    // - assign stuff from move_data to move_board
    for (std::size_t i = 0; i < move_data.centroids.size(); i++) {
        Space node;
        node.id = static_cast<int>(i);
        node.neighbors = spatial_graph[node.id];
        node.cloud = move_data.clusters[i];
        node.centroid = move_data.centroids[i];
        board_utils::assignEdges(spatial_graph, node);
        board.movement_spaces.insert({node.id, node});
    }

    // - assign stuff from repair_data to repair_board
    for (std::size_t i = 0; i < repair_data.centroids.size(); i++) {
        RepairVolume node;
        node.id = static_cast<int>(i);
        node.cloud = repair_data.clusters[i];
        node.centroid = repair_data.centroids[i];
        board.repair_spaces.insert({node.id, node});
        board.movement_spaces.at(assignRepairEdge(repair_data.centroids[i], move_data.centroids)).repair_edges.push_back(static_cast<int>(i));
    }

    std::cout << "[Board] Generated a game board with " << move_data.centroids.size() << " movement volumes and " << repair_data.centroids.size() << " repair volumes." << std::endl;
    return board;
}

Board generateBoard (const OctreeData &move_data, const OctreeData &repair_data)
{
    SpatialGraph spatial_graph {generateSpatialGraph(move_data)}; 
    return generateBoard(move_data, spatial_graph, repair_data);
}

Board generateBoard (const sensor_msgs::PointCloud2 &move_cloud, const float &move_octree_resolution, const sensor_msgs::PointCloud2 &repair_cloud, const float &repair_octree_resolution)
{
    OctreeData move_data;
    move_data.name = "moves";
    extractOctreeData(move_cloud, move_octree_resolution, move_data);
    SpatialGraph spatial_graph {generateSpatialGraph(move_data)}; 
    OctreeData repair_data;
    repair_data.name = "repairs";
    extractOctreeData(repair_cloud, repair_octree_resolution, repair_data);
    return generateBoard(move_data, spatial_graph, repair_data);
}

MoveBoard generateMoveBoard (const OctreeData &data, SpatialGraph &spatial_graph)
{
    MoveBoard board;

    // - Loop through the centroids
    for (std::size_t i = 0; i < data.centroids.size(); i++) {
        Space node;
        node.id = static_cast<int>(i);
        node.neighbors = spatial_graph[node.id];
        node.cloud = data.clusters[i];
        node.centroid = data.centroids[i];
        board_utils::assignEdges(spatial_graph, node);
        board.insert({node.id, node});
    }
    std::cout << "[Board] Generated a game movement board for " << data.centroids.size() << " input voxels!" << std::endl;
    return board;
}

MoveBoard generateMoveBoard (const OctreeData &data)
{
    SpatialGraph spatial_graph {generateSpatialGraph(data)}; 
    return generateMoveBoard(data, spatial_graph);
}

MoveBoard generateMoveBoard (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution)
{
    OctreeData data;
    data.name = "board";
    extractOctreeData(cloud, octree_resolution, data);
    SpatialGraph spatial_graph {generateSpatialGraph(data)}; 
    return generateMoveBoard(data, spatial_graph);
}

int assignRepairEdge (const PointT &centroid, const std::vector<PointT, Eigen::aligned_allocator<PointT>> &centroids)
{
    double min_dist_sqrd {INFINITY};
    int min_index;
    for (std::size_t i = 0; i < centroids.size(); i++) {
        double comp_dist_sqrd {pow(centroids[i].x - centroid.x, 2) + pow(centroids[i].y - centroid.y, 2)};
        if ((pow(centroids[i].x - centroid.x, 2) + pow(centroids[i].y - centroid.y, 2)) < min_dist_sqrd) {
            min_dist_sqrd = comp_dist_sqrd;
            min_index = static_cast<int>(i);
        }
    }
    return min_index;
}

void assignEdges (SpatialGraph &spatial_graph, Space &node)
{
    // - Check whether node is on the ground or not
    if (spatial_graph[node.id].z_neg == -1) {
        node.is_ground_level = true;
        // - Gantry edges
        assignGantryEdges(spatial_graph, node);
        // - Quadruped edges
        assignQuadrupedEdges(spatial_graph, node);
    }
    // - Drone edges
    assignDroneEdges(spatial_graph, node);
}

void assignGantryEdges (SpatialGraph &spatial_graph, Space &node)
{
    std::vector<int> candidates {
        // x & y direct neighbors
        spatial_graph[node.id].x_pos, 
        spatial_graph[node.id].x_neg, 
        spatial_graph[node.id].y_pos, 
        spatial_graph[node.id].y_neg,
    };
    for (std::size_t i = 0; i < candidates.size(); i++) {
        if (candidates[i] != -1) {
            if (!(std::find(node.gantry_edges.begin(), node.gantry_edges.end(), candidates[i]) != node.gantry_edges.end())) {
                node.gantry_edges.push_back(candidates[i]);
            }
        }
    }
}

void assignQuadrupedEdges (SpatialGraph &spatial_graph, Space &node)
{
    std::vector<int> candidates {
        // x & y direct neighbors
        spatial_graph[node.id].x_pos, 
        spatial_graph[node.id].x_neg, 
        spatial_graph[node.id].y_pos, 
        spatial_graph[node.id].y_neg,
        // planar diagonal neighbors
        spatial_graph[spatial_graph[node.id].x_pos].y_pos,
        spatial_graph[spatial_graph[node.id].x_pos].y_neg,
        spatial_graph[spatial_graph[node.id].x_neg].y_pos,
        spatial_graph[spatial_graph[node.id].x_neg].y_neg,
        spatial_graph[spatial_graph[node.id].y_pos].x_pos,
        spatial_graph[spatial_graph[node.id].y_pos].x_neg,
        spatial_graph[spatial_graph[node.id].y_neg].x_pos,
        spatial_graph[spatial_graph[node.id].y_neg].x_neg,
    };
    for (std::size_t i = 0; i < candidates.size(); i++) {
        if (candidates[i] != -1) {
            if (!(std::find(node.quadruped_edges.begin(), node.quadruped_edges.end(), candidates[i]) != node.quadruped_edges.end())) {
                node.quadruped_edges.push_back(candidates[i]);
            }
        }
    }
}

void assignDroneEdges (SpatialGraph &spatial_graph, Space &node)
{
    std::vector<int> candidates {
        // x & y direct neighbors
        spatial_graph[node.id].x_pos, 
        spatial_graph[node.id].x_neg, 
        spatial_graph[node.id].y_pos, 
        spatial_graph[node.id].y_neg,
        // planar diagonal neighbors
        spatial_graph[spatial_graph[node.id].x_pos].y_pos,
        spatial_graph[spatial_graph[node.id].x_pos].y_neg,
        spatial_graph[spatial_graph[node.id].x_neg].y_pos,
        spatial_graph[spatial_graph[node.id].x_neg].y_neg,
        spatial_graph[spatial_graph[node.id].y_pos].x_pos,
        spatial_graph[spatial_graph[node.id].y_pos].x_neg,
        spatial_graph[spatial_graph[node.id].y_neg].x_pos,
        spatial_graph[spatial_graph[node.id].y_neg].x_neg,
        // lower/higher plane neighbors
        spatial_graph[node.id].z_pos,
        spatial_graph[node.id].z_neg,
        spatial_graph[spatial_graph[node.id].x_pos].z_pos,
        spatial_graph[spatial_graph[node.id].x_pos].z_neg,
        spatial_graph[spatial_graph[node.id].x_neg].z_pos,
        spatial_graph[spatial_graph[node.id].x_neg].z_neg,
        spatial_graph[spatial_graph[node.id].y_pos].z_pos,
        spatial_graph[spatial_graph[node.id].y_pos].z_neg,
        spatial_graph[spatial_graph[node.id].y_neg].z_pos,
        spatial_graph[spatial_graph[node.id].y_neg].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_pos].y_pos].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_pos].y_neg].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_neg].y_pos].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_neg].y_neg].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_pos].x_pos].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_pos].x_neg].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_neg].x_pos].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_neg].x_neg].z_pos,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_pos].y_pos].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_pos].y_neg].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_neg].y_pos].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].x_neg].y_neg].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_pos].x_pos].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_pos].x_neg].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_neg].x_pos].z_neg,
        spatial_graph[spatial_graph[spatial_graph[node.id].y_neg].x_neg].z_neg,
    };

    for (std::size_t i = 0; i < candidates.size(); i++) {
        if (candidates[i] != -1) {
            if (!(std::find(node.drone_edges.begin(), node.drone_edges.end(), candidates[i]) != node.drone_edges.end())) {
                node.drone_edges.push_back(candidates[i]);
            }
        }
    }
}

SpatialGraph generateSpatialGraph (const OctreeData &data)
{
    SpatialGraph board;

    // - Loop through all centroids in octree data and find closest neighbor in positive and negative xyz directions
    for (std::size_t i = 0; i < data.centroids.size(); i++) {
        board.insert({static_cast<int>(i), board_utils::findNeighbors(data, static_cast<int>(i))});
    }
    std::cout << "[Board] Generated a spatial graph for " << data.centroids.size() << " input voxels." << std::endl;
    return board;
}

SpatialNode findNeighbors (const OctreeData &data, const int &index, const double &tolerance)
{
    SpatialNode node;

    const PointT comp = data.centroids[index];
    
    const Eigen::Vector3d x_basis (1, 0, 0);
    const Eigen::Vector3d y_basis (0, 1, 0);
    const Eigen::Vector3d z_basis (0, 0, 1);

    for (std::size_t i = 0; i < data.centroids.size(); i++) {
        // - Check if looking at target index, continue if so
        if (static_cast<int>(i) == index) {
            continue;
        }

        Eigen::Vector3d displacement (data.centroids[i].x - comp.x, data.centroids[i].y - comp.y, data.centroids[i].z - comp.z);
        
        // - Check if distance is greater than tolerance, continue if so
        float distance {static_cast<float>((pow(displacement(0), 2) + pow(displacement(1), 2) + pow(displacement(2), 2)))};
        if (distance > pow(data.discretization * 1.2 / cos(tolerance), 2)) {
            continue;
        }

        // - Calculate dot products and associate with a direction
        std::vector<double> dot_products {displacement.dot(x_basis), displacement.dot(y_basis), displacement.dot(z_basis)};

        double max_dot {dot_products[0]};
        int max_dot_index {0};
        for (std::size_t j = 1; j < dot_products.size(); j++) {
            if (abs(dot_products[j]) > abs(max_dot)) {
                max_dot = dot_products[j];
                max_dot_index = static_cast<int>(j);
            } 
        }

        switch (max_dot_index) {
            case 0: // x-direction
            {
                // - Check if within angular tolerance
                float theta = atan((sqrt(pow(displacement(1), 2) + pow(displacement(2), 2))) / abs(displacement(0)));
                if (theta > tolerance) {continue;}

                if (max_dot >= 0) {
                    if (node.x_pos_d == 0.) {
                        node.x_pos_d = distance;
                        node.x_pos = static_cast<int>(i);
                    } else if (distance < node.x_pos_d) {
                        node.x_pos_d = distance;
                        node.x_pos = static_cast<int>(i);
                    }
                } else if (max_dot < 0) {
                    if (node.x_neg_d == 0.) {
                        node.x_neg_d = distance;
                        node.x_neg = static_cast<int>(i);
                    } else if (distance < node.x_neg_d) {
                        node.x_neg_d = distance;
                        node.x_neg = static_cast<int>(i);
                    }
                }
                break;
            }
            case 1: // y-direction
            {
                // - Check if within angular tolerance
                float theta = atan((sqrt(pow(displacement(0), 2) + pow(displacement(2), 2))) / abs(displacement(1)));
                if (theta > tolerance) {continue;}

                if (max_dot >= 0) {
                    if (node.y_pos_d == 0.) {
                        node.y_pos_d = distance;
                        node.y_pos = static_cast<int>(i);
                    } else if (distance < node.y_pos_d) {
                        node.y_pos_d = distance;
                        node.y_pos = static_cast<int>(i);
                    }
                } else if (max_dot < 0) {
                    if (node.y_neg_d == 0.) {
                        node.y_neg_d = distance;
                        node.y_neg = static_cast<int>(i);
                    } else if (distance < node.y_neg_d) {
                        node.y_neg_d = distance;
                        node.y_neg = static_cast<int>(i);
                    }
                }
                break;
            }
            case 2: // z-direction
            {
                // - Check if within angular tolerance
                float theta = atan((sqrt(pow(displacement(0), 2) + pow(displacement(1), 2))) / abs(displacement(2)));
                if (theta > tolerance) {continue;}

                if (max_dot >= 0) {
                    if (node.z_pos_d == 0.) {
                        node.z_pos_d = distance;
                        node.z_pos = static_cast<int>(i);
                    } else if (distance < node.z_pos_d) {
                        node.z_pos_d = distance;
                        node.z_pos = static_cast<int>(i);
                    }
                } else if (max_dot < 0) {
                    if (node.z_neg_d == 0.) {
                        node.z_neg_d = distance;
                        node.z_neg = static_cast<int>(i);
                    } else if (distance < node.z_neg_d) {
                        node.z_neg_d = distance;
                        node.z_neg = static_cast<int>(i);
                    }
                }
                break;
            }
        }
    }
    return node;
}

void extractOctreeData (const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output)
{
    output.discretization = octree_resolution;

    PointCloud::Ptr input (new PointCloud);
    pcl::fromROSMsg(cloud, *input);

    pcl::octree::OctreePointCloudPointVector<PointT> octree (octree_resolution);
    octree.defineBoundingBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

    octree.setInputCloud(input);
    octree.addPointsFromInputCloud();

    octree.getOccupiedVoxelCenters(output.centroids);

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
    std::cout << "[Board] Extracted " << output.clusters.size() << " clusters and " << output.centroids.size() << " centroids from the provided cloud using a resolution of " << octree_resolution << " m." << std::endl;
}

PointT colorPairRandomUniform (PointCloud::Ptr &cloud, const PointT &centroid)
{
    std::random_device rd;
    std::mt19937 gen(rd());
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

void saveOctreeDataClouds (const OctreeData &input, const std::string &dir)
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
    writer.write(dir + input.name + "_clusters.pcd", *volumes, false);
    centroids->height = 1;
    centroids->width = centroids->points.size();
    writer.write(dir + input.name + "_centroids.pcd", *centroids, false);
    std::cout << "[Board] Finished coloring and saving clouds!" << std::endl;
}

void loadCloudasMsg (const std::string &dir, sensor_msgs::PointCloud2 &msg)
{
    pcl::PCDReader reader;
    PointCloud::Ptr cloud (new PointCloud);
    reader.read(dir, *cloud);
    std::cout << "[Board] Loaded cloud has " << cloud->points.size() << " points." << std::endl;
    pcl::toROSMsg(*cloud, msg);
}

}

#endif // GAME_BOARD_H
