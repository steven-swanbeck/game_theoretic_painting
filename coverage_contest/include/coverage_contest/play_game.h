#ifndef GAME_PLAYER_H
#define GAME_PLAYER_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "board.hpp"
#include "agents.hpp"
#include "game_manager.h"
#include "mcts.h"
#include "game_visualizer.h"

class GamePlayer
{
public:
    GamePlayer();
private:
    void loadGame ();
    bool playRandomGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool testMCTS (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void takeTurnMCTS ();
    bool testMarker (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void visualizeTurn (TurnSequence sequence);

    ros::NodeHandle nh_;

    ros::ServiceServer play_random_game_server_;
    ros::ServiceServer test_mcts_server_;
    ros::Publisher gantry_visualizer_;
    ros::Publisher quadruped_visualizer_;
    ros::Publisher drone_visualizer_;
    ros::ServiceServer test_marker_server_;
    
    sensor_msgs::PointCloud2 map_;
    sensor_msgs::PointCloud2 marked_;

    GameManager manager_;
    Board board_;
    agents::Party party_;
    GameVisualizer *visualizer_;
};

#endif // GAME_PLAYER_H
