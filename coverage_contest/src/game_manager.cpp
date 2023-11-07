#include "coverage_contest/game_manager.h"

// TODO add game dynamics into this file

GameManager::GameManager ()
{
    
}

void GameManager::instantiateBoard (const std::string &dir, const float &discretization)
{
    sensor_msgs::PointCloud2 map;
    board_utils::loadCloudasMsg(dir, map);
    board_ = board_utils::generateBoard(map, discretization);
}

void GameManager::instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries)
{
    party_ = agents::instantiatePlayers(num_drones, num_quadrupeds, num_gantries);
    player_turn_ = 0;
}

void GameManager::generateTurnOrder ()
{
    agents::randomShufflePlayingOrder (party_);
    player_turn_ = 0;
}

std::string GameManager::upNext () 
{
    player_turn_++;
    if (player_turn_ > (party_.playing_order.size() - 1)) {
        player_turn_ = 0;
    }
    std::cout << "[Manager] Player up next is: " << party_.playing_order[player_turn_] << std::endl;
    std::cout << "[Manager] Up next is " << party_.players.at(party_.playing_order[player_turn_]).id << std::endl;
    return party_.playing_order[player_turn_];
}

void GameManager::playRandomMove ()
{

    // - use some amount of movement, if available where the robot ended up, take some action
}

// ! note that we probably can't simulate an entire game here, we probably instead want to forecast ahead only a set number of turns (maybe 5 or so) for each player and make a value function that assesses how they've performed in those turns
void GameManager::simulateRandomGame ()
{
    // - cycle through all players and play random moves for all until all surfaces are covered
}

// ! this can be that function
void GameManager::assessValue ()
{

}

bool GameManager::isOver ()
{

}

// TODO figure out how to template this!
// template <class AgentT>
// std::vector<int> GameManager::nextMoves (AgentT agent)
// {

// }
