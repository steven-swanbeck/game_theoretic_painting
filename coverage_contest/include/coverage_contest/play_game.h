#ifndef GAME_PLAYER_H
#define GAME_PLAYER_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include "board.hpp"
#include "agents.hpp"

class GamePlayer
{
public:
    GamePlayer();
private:
    ros::NodeHandle nh_;
    
    sensor_msgs::PointCloud2 map_;
    sensor_msgs::PointCloud2 marked_;

    Board board_;
    agents::Players players_;
};

#endif // GAME_PLAYER_H
