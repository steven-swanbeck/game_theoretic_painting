#ifndef GAME_BOARD_H
#define GAME_BOARD_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <random>
#include <math.h>
#include "agents.hpp"

// . Data Structures
/** @struct OctreeData
 * @brief used to store extracted octree data for graph construction
 */
struct OctreeData
{
    std::string name;
    float discretization;
    std::vector<sensor_msgs::PointCloud2> clusters;
    std::vector<PointT, Eigen::aligned_allocator<PointT>> centroids;
};

// . Helpful spatial relations
/** @struct SpatialNode
 * @brief contains positive and negative nearest node ids in x, y, and z directions and their Euclidean distance
 */
struct SpatialNode
{
    int x_pos{-1};
    float x_pos_d{0};
    int x_neg{-1};
    float x_neg_d{0};
    int y_pos{-1};
    float y_pos_d{0};
    int y_neg{-1};
    float y_neg_d{0};
    int z_pos{-1};
    float z_pos_d{0};
    int z_neg{-1};
    float z_neg_d{0};
};
using SpatialGraph = std::map<int, SpatialNode>;

// . Movement game board
/** @struct Space
 * @brief contains relevant info related to a specific movement-discretized region in space
 * @var Space::id
 * 'id' matches the key used in a higher map to access the volume
 * @var Space::centroid
 * 'centroid' contains the geometric center of the repair volume
 * @var Space::cloud
 * 'cloud' contains the associated surface geometry to be repaired
 * @var Space::gantry_edges
 * 'gantry_edges' denotes the id's of edges that can be reached by a gantry-type system
 * @var Space::quadruped_edges
 * 'quadruped_edges' denotes the id's of edges that can be reached by a quadruped-type system
 * @var Space::drone_edges
 * 'drone_edges' denotes the id's of edges that can be reached by a drone-type system
 * @var Space::is_ground_level
 * 'is_ground_level' denotes whether the volume is at ground level
 * @var Space::repair_edges
 * 'repair_edges' contains the id's of repair volumes located within the Space
 */
struct Space
{
    int id;
    SpatialNode neighbors;
    std::vector<int> gantry_edges;
    std::vector<int> quadruped_edges;
    std::vector<int> drone_edges;
    PointT centroid;
    sensor_msgs::PointCloud2 cloud;
    bool is_ground_level{false};
    std::vector<int> repair_edges;
};
using MoveBoard = std::map<int, Space>;

// . Repair action board
/** @struct RepairVolume
 * @brief contains relevant info related to a specific repair-discretized region in space
 * @var RepairVolume::id
 * 'id' matches the key used in a higher map to access the volume
 * @var RepairVolume::centroid
 * 'centroid' contains the geometric center of the repair volume
 * @var RepairVolume::cloud
 * 'cloud' contains the associated surface geometry to be repaired
 * @var RepairVolume::covered
 * 'covered' denotes wheteher a volume has already been repaired
 */
struct RepairVolume
{
    int id;
    PointT centroid;
    sensor_msgs::PointCloud2 cloud;
    bool covered{false};
};
using RepairBoard = std::map<int, RepairVolume>;

// . Overall board struct
/** @struct Board
 * @brief contains a map of Robot players with keys that match the elements of a vector that tracks playing order
 * @var Board::movement_spaces
 * 'movement_spaces' contains the static movement graph of the current game
 * @var Board::repair_spaces
 * 'repair_spaces' contains the dynamic repair graph of the current game
 */
struct Board
{
    MoveBoard movement_spaces;
    RepairBoard repair_spaces;
};

// . Possible turns struct
/** @struct Action
 * @brief denotes a movement or repair id associated with an action to be taken
 * @var Action::move_id
 * 'move_id' denotes a node id from the movement graph to move to
 * @var Action::repair_id
 * 'repair_id' denotes a node id from the repair graph to repair
 */
struct Action
{
    int move_id{-1};
    int repair_id{-1};
};
using MoveOptions = std::vector<Action>;
using TurnSequence = std::queue<Action>;
using TurnOptions = std::vector<TurnSequence>;

namespace board_utils
{

    // . Function Prototypes
    /** Extracts octree data from a given point cloud
     * @brief extract octree data from a given point cloud
     * @param cloud point cloud of interest
     * @param octree_resolution octree lowest-level discretization
     * @param output octree data storage type to be filled
     */
    void extractOctreeData(const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output);

    /** Assigns spatial edges to a graph node
     * @brief assigns spatial edges to a node
     * @param spatial_graph graph of nodes for which to generate relations
     * @param node space for which to assign edges
     */
    void assignEdges(const SpatialGraph &spatial_graph, Space &node);

    /** Colors a centroid and cloud data the same random color
     * @brief randomly colors a centroid and cloud
     * @param cloud cloud to be colored
     * @param centroid centroid to be colored
     * @return colored centroid point
     */
    PointT colorPairRandomUniform(PointCloud::Ptr &cloud, const PointT &centroid);

    /** Colors a cloud a random color
     * @brief Colors a cloud a random color
     * @param cloud cloud to be colored
     */
    void colorCloudRandomUniform(PointCloud::Ptr &cloud);

    /** Creates randomly colored clouds to show the movment and repair spaces of a board
     * @brief Creates colorful movement and repair clouds for visualization
     * @param board board from which cloud information is extracted
     * @param map_cloud cloud msg for map modified in place
     * @param marked_cloud cloud msg for repair modified in place
     */
    void buildColoredClouds (Board &board, sensor_msgs::PointCloud2 &map_cloud, sensor_msgs::PointCloud2 &marked_cloud);

    /** Saves octree data in easily-visualizable format
     * @brief saves octree data into mulit-colored clouds
     * @param input octree data to be converted
     * @param dir write directory for output clouds
     */
    void saveOctreeDataClouds(const OctreeData &input, const std::string &dir);

    /** Loads PCD file as sensor_msgs::PointCloud2
     * @brief loads pcl file as ros msg
     * @param dir read directory for input cloud
     * @param msg msg to be filled
     */
    void loadCloudasMsg(const std::string &dir, sensor_msgs::PointCloud2 &msg);

    /** Assigns spatial edges to a graph based on distance
     * @brief assigns spatial edges to a graph
     * @param data octree data used to populate graph
     * @param index id of current node
     * @param tolerance azimuth of tolerance cone within which to search for neighbors in a particular direction
     */
    SpatialNode findNeighbors(const OctreeData &data, const int &index, const double &tolerance = 0.174533);

    /** Generates a spatial graph
     * @brief generates a spatial graph given input octree data
     * @param data octree data used to populate graph
     */
    SpatialGraph generateSpatialGraph(const OctreeData &data);

    /** Assigns robot-specific edges to a graph node
     * @brief generates a spatial graph given input octree data
     * @param spatial_graph spatial graph used to populate node
     * @param node movement space to be populated
     */
    void assignEdges(SpatialGraph &spatial_graph, Space &node);
    void assignGantryEdges(SpatialGraph &spatial_graph, Space &node);
    void assignQuadrupedEdges(SpatialGraph &spatial_graph, Space &node);
    void assignDroneEdges(SpatialGraph &spatial_graph, Space &node);

    /** Assigns repair edges to a graph node
     * @brief assigns repair edges to a graph node
     * @param centroid centroid of repair volume, used to find owner with shortest Euclidean distance
     * @param node movement space to be populated
     */
    int assignRepairEdge(const PointT &centroid, const std::vector<PointT, Eigen::aligned_allocator<PointT>> &centroids);

    /** Generates a movement board
     * @brief generates a movement board
     * @param data octree data used to construct movement graph
     * @param spatial_graph spatial data used to construct movement graph
     * @param cloud cloud from which to generate movement graph
     * @param octree_discretization discretization used to generate movement graph
     * @return MoveBoard type
     */
    MoveBoard generateMoveBoard(const OctreeData &data, SpatialGraph &spatial_graph);
    MoveBoard generateMoveBoard(const OctreeData &data);
    MoveBoard generateMoveBoard(const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution);

    /** Generates a game board
     * @brief generates a game board
     * @param move_data octree data used to construct movement graph
     * @param spatial_graph spatial data used to construct movement graph
     * @param repair_data octree data used to construct repair graph
     * @param move_cloud cloud from which to generate movement graph
     * @param move_octree_discretization discretization used to generate movement graph
     * @param repair_cloud cloud from which to generate repair graph
     * @param repair_octree_discretization discretization used to generate repair graph
     * @return MoveBoard type
     */
    Board generateBoard(const OctreeData &move_data, SpatialGraph &spatial_graph, const OctreeData &repair_data);
    Board generateBoard(const OctreeData &move_data, const OctreeData &repair_data);
    Board generateBoard(const sensor_msgs::PointCloud2 &move_cloud, const float &move_octree_resolution, const sensor_msgs::PointCloud2 &repair_cloud, const float &repair_octree_resolution);

    // . Function Definitions
    void buildColoredClouds (Board &board, sensor_msgs::PointCloud2 &map_cloud, sensor_msgs::PointCloud2 &marked_cloud)
    {
        PointCloud::Ptr map (new PointCloud);
        PointCloud::Ptr marked (new PointCloud);
        pcl::fromROSMsg(map_cloud, *map);
        pcl::fromROSMsg(marked_cloud, *marked);

        // - iterate over board and generate randomly colored clouds for each region
        MoveBoard::iterator it;
        std::cout << "starting loop through move:" << std::endl;
        for (it = board.movement_spaces.begin(); it != board.movement_spaces.end(); it++) {
            PointCloud::Ptr board_space (new PointCloud);
            pcl::fromROSMsg(it->second.cloud, *board_space);
            colorCloudRandomUniform(board_space);
            *map += *board_space;
        }

        std::cout << "starting loop through repair:" << std::endl;
        RepairBoard::iterator itr;
        for (itr = board.repair_spaces.begin(); itr != board.repair_spaces.end(); itr++) {
            PointCloud::Ptr board_volume (new PointCloud);
            pcl::fromROSMsg(itr->second.cloud, *board_volume);
            colorCloudRandomUniform(board_volume);
            *marked += *board_volume;
        }

        std::cout << "converting to msgs:" << std::endl;
        pcl::toROSMsg(*map, map_cloud);
        pcl::toROSMsg(*marked, marked_cloud);
    }

    void colorCloudRandomUniform(PointCloud::Ptr &cloud)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(0, 255);
        int r{distr(gen)};
        int g{distr(gen)};
        int b{distr(gen)};

        for (std::size_t i = 0; i < cloud->points.size(); i++)
        {
            cloud->points[i].r = r;
            cloud->points[i].g = g;
            cloud->points[i].b = b;
        }
    }

    Board generateBoard(const OctreeData &move_data, SpatialGraph &spatial_graph, const OctreeData &repair_data)
    {
        Board board;

        // - assign stuff from move_data to move_board
        for (std::size_t i = 0; i < move_data.centroids.size(); i++)
        {
            Space node;
            node.id = static_cast<int>(i);
            node.neighbors = spatial_graph[node.id];
            node.cloud = move_data.clusters[i];
            node.centroid = move_data.centroids[i];
            board_utils::assignEdges(spatial_graph, node);
            board.movement_spaces.insert({node.id, node});
        }

        // - assign stuff from repair_data to repair_board
        for (std::size_t i = 0; i < repair_data.centroids.size(); i++)
        {
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

    Board generateBoard(const OctreeData &move_data, const OctreeData &repair_data)
    {
        SpatialGraph spatial_graph{generateSpatialGraph(move_data)};
        return generateBoard(move_data, spatial_graph, repair_data);
    }

    Board generateBoard(const sensor_msgs::PointCloud2 &move_cloud, const float &move_octree_resolution, const sensor_msgs::PointCloud2 &repair_cloud, const float &repair_octree_resolution)
    {
        OctreeData move_data;
        move_data.name = "moves";
        extractOctreeData(move_cloud, move_octree_resolution, move_data);
        SpatialGraph spatial_graph{generateSpatialGraph(move_data)};
        OctreeData repair_data;
        repair_data.name = "repairs";
        extractOctreeData(repair_cloud, repair_octree_resolution, repair_data);
        return generateBoard(move_data, spatial_graph, repair_data);
    }

    MoveBoard generateMoveBoard(const OctreeData &data, SpatialGraph &spatial_graph)
    {
        MoveBoard board;

        // - Loop through the centroids
        for (std::size_t i = 0; i < data.centroids.size(); i++)
        {
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

    MoveBoard generateMoveBoard(const OctreeData &data)
    {
        SpatialGraph spatial_graph{generateSpatialGraph(data)};
        return generateMoveBoard(data, spatial_graph);
    }

    MoveBoard generateMoveBoard(const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution)
    {
        OctreeData data;
        data.name = "board";
        extractOctreeData(cloud, octree_resolution, data);
        SpatialGraph spatial_graph{generateSpatialGraph(data)};
        return generateMoveBoard(data, spatial_graph);
    }

    int assignRepairEdge(const PointT &centroid, const std::vector<PointT, Eigen::aligned_allocator<PointT>> &centroids)
    {
        double min_dist_sqrd{INFINITY};
        int min_index;
        for (std::size_t i = 0; i < centroids.size(); i++)
        {
            double comp_dist_sqrd{pow(centroids[i].x - centroid.x, 2) + pow(centroids[i].y - centroid.y, 2)};
            if ((pow(centroids[i].x - centroid.x, 2) + pow(centroids[i].y - centroid.y, 2)) < min_dist_sqrd)
            {
                min_dist_sqrd = comp_dist_sqrd;
                min_index = static_cast<int>(i);
            }
        }
        return min_index;
    }

    void assignEdges(SpatialGraph &spatial_graph, Space &node)
    {
        // - Check whether node is on the ground or not
        if (spatial_graph[node.id].z_neg == -1)
        {
            node.is_ground_level = true;
            // - Gantry edges
            assignGantryEdges(spatial_graph, node);
            // - Quadruped edges
            assignQuadrupedEdges(spatial_graph, node);
        }
        // - Drone edges
        assignDroneEdges(spatial_graph, node);
    }

    void assignGantryEdges(SpatialGraph &spatial_graph, Space &node)
    {
        std::vector<int> candidates{
            // x & y direct neighbors
            spatial_graph[node.id].x_pos,
            spatial_graph[node.id].x_neg,
            spatial_graph[node.id].y_pos,
            spatial_graph[node.id].y_neg,
        };
        for (std::size_t i = 0; i < candidates.size(); i++)
        {
            if (candidates[i] != -1)
            {
                if (!(std::find(node.gantry_edges.begin(), node.gantry_edges.end(), candidates[i]) != node.gantry_edges.end()))
                {
                    node.gantry_edges.push_back(candidates[i]);
                }
            }
        }
    }

    void assignQuadrupedEdges(SpatialGraph &spatial_graph, Space &node)
    {
        std::vector<int> candidates{
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
        for (std::size_t i = 0; i < candidates.size(); i++)
        {
            if (candidates[i] != -1)
            {
                if (!(std::find(node.quadruped_edges.begin(), node.quadruped_edges.end(), candidates[i]) != node.quadruped_edges.end()))
                {
                    node.quadruped_edges.push_back(candidates[i]);
                }
            }
        }
    }

    void assignDroneEdges(SpatialGraph &spatial_graph, Space &node)
    {
        std::vector<int> candidates{
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

        for (std::size_t i = 0; i < candidates.size(); i++)
        {
            if (candidates[i] != -1)
            {
                if (!(std::find(node.drone_edges.begin(), node.drone_edges.end(), candidates[i]) != node.drone_edges.end()))
                {
                    node.drone_edges.push_back(candidates[i]);
                }
            }
        }
    }

    SpatialGraph generateSpatialGraph(const OctreeData &data)
    {
        SpatialGraph board;

        // - Loop through all centroids in octree data and find closest neighbor in positive and negative xyz directions
        for (std::size_t i = 0; i < data.centroids.size(); i++)
        {
            board.insert({static_cast<int>(i), board_utils::findNeighbors(data, static_cast<int>(i))});
        }
        std::cout << "[Board] Generated a spatial graph for " << data.centroids.size() << " input voxels." << std::endl;
        return board;
    }

    SpatialNode findNeighbors(const OctreeData &data, const int &index, const double &tolerance)
    {
        SpatialNode node;

        const PointT comp = data.centroids[index];

        const Eigen::Vector3d x_basis(1, 0, 0);
        const Eigen::Vector3d y_basis(0, 1, 0);
        const Eigen::Vector3d z_basis(0, 0, 1);

        for (std::size_t i = 0; i < data.centroids.size(); i++)
        {
            // - Check if looking at target index, continue if so
            if (static_cast<int>(i) == index)
            {
                continue;
            }

            Eigen::Vector3d displacement(data.centroids[i].x - comp.x, data.centroids[i].y - comp.y, data.centroids[i].z - comp.z);

            // - Check if distance is greater than tolerance, continue if so
            float distance{static_cast<float>((pow(displacement(0), 2) + pow(displacement(1), 2) + pow(displacement(2), 2)))};
            if (distance > pow(data.discretization * 1.2 / cos(tolerance), 2))
            {
                continue;
            }

            // - Calculate dot products and associate with a direction
            std::vector<double> dot_products{displacement.dot(x_basis), displacement.dot(y_basis), displacement.dot(z_basis)};

            double max_dot{dot_products[0]};
            int max_dot_index{0};
            for (std::size_t j = 1; j < dot_products.size(); j++)
            {
                if (abs(dot_products[j]) > abs(max_dot))
                {
                    max_dot = dot_products[j];
                    max_dot_index = static_cast<int>(j);
                }
            }

            switch (max_dot_index)
            {
            case 0: // x-direction
            {
                // - Check if within angular tolerance
                float theta = atan((sqrt(pow(displacement(1), 2) + pow(displacement(2), 2))) / abs(displacement(0)));
                if (theta > tolerance)
                {
                    continue;
                }

                if (max_dot >= 0)
                {
                    if (node.x_pos_d == 0.)
                    {
                        node.x_pos_d = distance;
                        node.x_pos = static_cast<int>(i);
                    }
                    else if (distance < node.x_pos_d)
                    {
                        node.x_pos_d = distance;
                        node.x_pos = static_cast<int>(i);
                    }
                }
                else if (max_dot < 0)
                {
                    if (node.x_neg_d == 0.)
                    {
                        node.x_neg_d = distance;
                        node.x_neg = static_cast<int>(i);
                    }
                    else if (distance < node.x_neg_d)
                    {
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
                if (theta > tolerance)
                {
                    continue;
                }

                if (max_dot >= 0)
                {
                    if (node.y_pos_d == 0.)
                    {
                        node.y_pos_d = distance;
                        node.y_pos = static_cast<int>(i);
                    }
                    else if (distance < node.y_pos_d)
                    {
                        node.y_pos_d = distance;
                        node.y_pos = static_cast<int>(i);
                    }
                }
                else if (max_dot < 0)
                {
                    if (node.y_neg_d == 0.)
                    {
                        node.y_neg_d = distance;
                        node.y_neg = static_cast<int>(i);
                    }
                    else if (distance < node.y_neg_d)
                    {
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
                if (theta > tolerance)
                {
                    continue;
                }

                if (max_dot >= 0)
                {
                    if (node.z_pos_d == 0.)
                    {
                        node.z_pos_d = distance;
                        node.z_pos = static_cast<int>(i);
                    }
                    else if (distance < node.z_pos_d)
                    {
                        node.z_pos_d = distance;
                        node.z_pos = static_cast<int>(i);
                    }
                }
                else if (max_dot < 0)
                {
                    if (node.z_neg_d == 0.)
                    {
                        node.z_neg_d = distance;
                        node.z_neg = static_cast<int>(i);
                    }
                    else if (distance < node.z_neg_d)
                    {
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

    void extractOctreeData(const sensor_msgs::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output)
    {
        output.discretization = octree_resolution;

        PointCloud::Ptr input(new PointCloud);
        pcl::fromROSMsg(cloud, *input);

        pcl::octree::OctreePointCloudPointVector<PointT> octree(octree_resolution);
        octree.defineBoundingBox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

        octree.setInputCloud(input);
        octree.addPointsFromInputCloud();

        octree.getOccupiedVoxelCenters(output.centroids);

        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
        {
            pcl::Indices indexVector;
            pcl::octree::OctreeNode *node = it.getCurrentOctreeNode();
            pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
            container.getPointIndices(indexVector);

            if (indexVector.size() < 1)
            {
                continue;
            }

            PointCloud::Ptr cluster(new PointCloud);

            for (const auto &i : indexVector)
            {
                cluster->points.push_back((*input)[i]);
            }

            sensor_msgs::PointCloud2 cluster_msg;
            pcl::toROSMsg(*cluster, cluster_msg);
            output.clusters.push_back(cluster_msg);
        }
        std::cout << "[Board] Extracted " << output.clusters.size() << " clusters and " << output.centroids.size() << " centroids from the provided cloud using a resolution of " << octree_resolution << " m." << std::endl;
    }

    PointT colorPairRandomUniform(PointCloud::Ptr &cloud, const PointT &centroid)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(0, 255);
        int r{distr(gen)};
        int g{distr(gen)};
        int b{distr(gen)};

        for (std::size_t i = 0; i < cloud->points.size(); i++)
        {
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

    void saveOctreeDataClouds(const OctreeData &input, const std::string &dir)
    {
        PointCloud::Ptr volumes(new PointCloud);
        PointCloud::Ptr centroids(new PointCloud);

        for (std::size_t i = 0; i < input.centroids.size(); i++)
        {
            PointCloud::Ptr cloud(new PointCloud);
            pcl::fromROSMsg(input.clusters[i], *cloud);

            PointT colored_centroid{colorPairRandomUniform(cloud, input.centroids[i])};
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

    void loadCloudasMsg(const std::string &dir, sensor_msgs::PointCloud2 &msg)
    {
        pcl::PCDReader reader;
        PointCloud::Ptr cloud(new PointCloud);
        reader.read(dir, *cloud);
        std::cout << "[Board] Loaded cloud has " << cloud->points.size() << " points." << std::endl;
        pcl::toROSMsg(*cloud, msg);
    }

}

#endif // GAME_BOARD_H
