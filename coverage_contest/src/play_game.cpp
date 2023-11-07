#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    manager_.instantiateBoard("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", 3.0);
    manager_.instantiatePlayers(1, 1, 1);
    
    manager_.upNext();
    manager_.upNext();
    manager_.upNext();
    manager_.upNext();
    manager_.upNext();

    ros::spin();
}

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
