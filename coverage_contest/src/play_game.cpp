#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    manager_.instantiateBoard("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", 3.0, "/home/steven/game_theoretic_painting/src/models/clouds/revised/marked.pcd", 1.0);
    // manager_.instantiatePlayers(1, 1, 1, 12);
    manager_.instantiatePlayers(1, 0, 0, 0);
    
    // manager_.listMovesfromNode();
    manager_.takeTurn();
    manager_.printMovesfromState(0);
    manager_.printMovesfromState(2);
    manager_.printMovesfromState(4);
    manager_.testRandomTurns(10);
    // manager_.startNext();
    // manager_.takeTurn();
    // manager_.startNext();
    // manager_.takeTurn();

    std::cout << "[Play game] Past game tests." << std::endl;
    ros::spin();
}

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
