#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    manager_.instantiateBoard("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", 3.0, "/home/steven/game_theoretic_painting/src/models/clouds/revised/marked.pcd", 1.0);
    // manager_.instantiatePlayers(1, 1, 1, 12);
    manager_.instantiatePlayers(0, 0, 1, 0);
    
    // manager_.listMovesfromNode();
    manager_.takeTurn();
    manager_.printMovesfromState(0);
    manager_.printMovesfromState(2);
    manager_.printMovesfromState(4);
    // manager_.startNext();
    // manager_.takeTurn();
    // manager_.startNext();
    // manager_.takeTurn();

    TurnGraph turn_graph {manager_.createTurnGraph ()};

    TurnGraph::iterator it;
    for (it = turn_graph.begin(); it != turn_graph.end(); it++) {
        std::cout << "Move id: " << it->first << std::endl;
        std::cout << "location: " << it->second.location << std::endl;
        std::cout << "\tparent: " << it->second.parent << std::endl;
        std::cout << "\tchildren: [";
        for (int i = 0; i < it->second.children.size(); i++) {
            std::cout << it->second.children[i] << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "\taction: (move: " << it->second.action.move_id << ", repair: " << it->second.action.repair_id << ")" << std::endl; 
    }

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
