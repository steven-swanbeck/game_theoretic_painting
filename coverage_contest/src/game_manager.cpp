#include "coverage_contest/game_manager.h"

// TODO add game dynamics into this file

GameManager::GameManager ()
{
    
}

void GameManager::instantiateBoard (const std::string &move_dir, const float &move_discretization, const std::string &repair_dir, const float &repair_discretization)
{
    sensor_msgs::PointCloud2 move_map;
    board_utils::loadCloudasMsg(move_dir, move_map);
    sensor_msgs::PointCloud2 repair_map;
    board_utils::loadCloudasMsg(repair_dir, repair_map);
    board_ = board_utils::generateBoard(move_map, move_discretization, repair_map, repair_discretization);

    // MoveBoard::iterator it;
    // for (it = board_.movement_spaces.begin(); it != board_.movement_spaces.end(); it++) {
    //     std::cout << "Id: " << it->first << " (" << it->second.id << ")" << std::endl;
    //     for (int i = 0; i < it->second.repair_edges.size(); i++) {
    //         std::cout << "\t" << it->second.repair_edges[i] << std::endl;
    //     }
    // }
}

void GameManager::instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries, int starting_position)
{
    party_ = agents::instantiatePlayers(num_drones, num_quadrupeds, num_gantries);
    player_turn_ = 0;
    total_turns_ = 0;
    for (std::size_t i = 0; i < party_.playing_order.size(); i++) {
        party_.players.at(party_.playing_order[i]).update_location(starting_position);
    }
    std::cout << "[Manager]\n--------------------\nStarting Party Turn " << total_turns_ << "\n--------------------" << std::endl;
    std::cout << "[Manager] " << party_.playing_order[player_turn_] << " is starting their turn." << std::endl;
}

void GameManager::generateTurnOrder ()
{
    agents::randomShufflePlayingOrder (party_);
    player_turn_ = 0;
}

std::string GameManager::startNext () 
{
    player_turn_++;
    if (player_turn_ > (party_.playing_order.size() - 1)) {
        player_turn_ = 0;
        total_turns_++;
        std::cout << "[Manager]\n--------------------\nStarting Party Turn " << total_turns_ << "\n--------------------" << std::endl;
    }
    std::cout << "[Manager] " << party_.playing_order[player_turn_] << " is starting their turn." << std::endl;
    // std::cout << "[Manager] Up next is " << party_.players.at(party_.playing_order[player_turn_]).get_id() << std::endl;
    return party_.playing_order[player_turn_];
}

std::string GameManager::playingNow () 
{
    // std::cout << "[Manager] Playing now is: " << party_.playing_order[player_turn_] << std::endl;
    return party_.playing_order[player_turn_];
}

// TODO
void GameManager::takeTurn ()
{
    if (!checkBattery()) {return;}

    PossibleMoves candidates {listMovesfromNode ()};
    for (int i = 0; i < candidates.size(); i++) {
        if (candidates[i].repair_id == -1) {
            std::cout << "\tMovement option: " << candidates[i].move_id << std::endl; 
        } else {
            std::cout << "\tRepair option: " << candidates[i].repair_id << std::endl; 
        }
    }
}

// TODO
void GameManager::playRandomMove ()
{
    
}

// TODO
PossibleMoves GameManager::listMovesfromNode (const int &index)
{
    PossibleMoves candidates;

    // - create a move for all movement options and repair options
    std::vector<int> movement_edges;
    std::vector<int> repair_edges {board_.movement_spaces.at(index).repair_edges};

    switch (party_.players.at(playingNow()).get_type()) {
        case 0: // drone
        {
            movement_edges = board_.movement_spaces.at(index).drone_edges;
            break;
        }
        case 1: // quadruped
        {
            movement_edges = board_.movement_spaces.at(index).quadruped_edges;
            break;
        }
        case 2: // gantry
        {
            movement_edges = board_.movement_spaces.at(index).gantry_edges;
            break;
        }
    }

    for (int i = 0; i < movement_edges.size(); i++) {
        Action action;
        action.move_id = movement_edges[i];
        candidates.push_back(action);
    }
    for (int i = 0; i < repair_edges.size(); i++) {
        Action action;
        action.repair_id = repair_edges[i];
        candidates.push_back(action);
    }

    return candidates;
}

PossibleMoves GameManager::listMovesfromNode ()
{
    return listMovesfromNode (party_.players.at(playingNow()).get_location());
}

// TODO
// ! note that we probably can't simulate an entire game here, we probably instead want to forecast ahead only a set number of turns (maybe 5 or so) for each player and make a value function that assesses how they've performed in those turns
void GameManager::simulateRandomGame ()
{
    // - cycle through all players and play random moves for all until all surfaces are covered
}

// TODO
// ! this can be that function
void GameManager::assessValue ()
{
    // - compare 
}

// TODO
bool GameManager::isOver ()
{

}

bool GameManager::checkBattery ()
{
    if (party_.players.at(playingNow()).remaining_battery > 0) {
        party_.players.at(playingNow()).remaining_battery--;
        return true;
    } else {
        party_.players.at(playingNow()).remaining_charge_time--;
        if (party_.players.at(playingNow()).remaining_charge_time <= 0) {
            party_.players.at(playingNow()).reset_remaining_battery();
            party_.players.at(playingNow()).reset_remaining_charge_time();
        }
        return false;
    }
}

// TODO figure out how to template this!
// template <class AgentT>
// std::vector<int> GameManager::nextMoves (AgentT agent)
// {

// }
