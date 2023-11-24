#ifndef GAME_PLAYER_H
#define GAME_PLAYER_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "board.hpp"
#include "agents.hpp"
#include "game_manager.h"
#include "mcts.h"
#include "game_visualizer.h"
#include <fstream>

class GamePlayer
{
public:
    GamePlayer();
private:
    void loadGame ();
    void loadGame (int n_drones, int n_quadrupeds, int n_gantries);
    bool playRandomGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool playGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void takeTurnMCTS (bool should_visualize=true);
    void playMCTSGame (bool should_visualize=true);
    bool testMarker (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void visualizeTurn (TurnSequence sequence);
    bool clearGameVisualizer (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void instantiateVisualizer ();
    void interpolatePath (const int &start_node, const int &end_node);
    bool playNGames (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool resetGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool exhaustiveSearch (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    std::vector<float_t> getLocationVector (const std::string &id);
    std::vector<float_t> getLocationVector (const int &id);
    std::vector<float_t> pointToVector (const PointT &point);

    ros::NodeHandle nh_;

    ros::ServiceServer play_random_game_server_;
    ros::ServiceServer play_game_server_;
    ros::ServiceServer play_n_games_server_;
    ros::ServiceServer exhaustive_search_server_;
    ros::Publisher gantry_visualizer_;
    ros::Publisher quadruped_visualizer_;
    ros::Publisher drone_visualizer_;
    ros::ServiceServer test_marker_server_;

    ros::Publisher map_visualizer_;
    ros::Publisher marked_visualizer_;
    ros::Publisher environment_visualizer_;

    ros::ServiceServer reset_game_;
    ros::ServiceServer clear_game_visualizer_;
    
    sensor_msgs::PointCloud2 map_;
    sensor_msgs::PointCloud2 marked_;
    // std::string environment_mesh_;
    // std::string drone_mesh_;
    // std::string quadruped_mesh_;
    // std::string gantry_mesh_;

    GameManager *manager_;
    GameVisualizer *visualizer_;
};

#endif // GAME_PLAYER_H
