#ifndef GAME_PLAYER_H
#define GAME_PLAYER_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "board.hpp"
#include "agents.hpp"
#include "game_manager.h"
#include "mcts.h"

class GamePlayer
{
public:
    GamePlayer();
private:
    void simulateGame ();

    ros::NodeHandle nh_;
    
    sensor_msgs::PointCloud2 map_;
    sensor_msgs::PointCloud2 marked_;

    GameManager manager_;
    Board board_;
    agents::Party party_;
    // MCTS mcts_;
};

#endif // GAME_PLAYER_H
