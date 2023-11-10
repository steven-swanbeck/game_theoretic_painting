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
// ! remember that I want the gantry to be able to paint stuff in both the volume it occupies and the volume above it!
void GameManager::takeTurn ()
{
    if (!sufficientBattery()) {return;}

    PossibleMoves candidates {listMovesfromNode()};
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


void GameManager::printMovesfromState (int state)
{
    std::cout << "Move candidates at state " << state << ":" << std::endl;
    PossibleMoves candidates {listMovesfromNode(state)};
    for (int i = 0; i < candidates.size(); i++) {
        if (candidates[i].repair_id == -1) {
            std::cout << "\tMovement option: " << candidates[i].move_id << std::endl; 
        } else {
            std::cout << "\tRepair option: " << candidates[i].repair_id << std::endl; 
        }
    }
}

// ! WILL NOT WORK
TurnGraph GameManager::createTurnGraph ()
{
    TurnGraph turn_graph;
    TurnNode current_node {0, -1, party_.players.at(playingNow()).get_location(), party_.players.at(playingNow()), board_.repair_spaces};
    current_node.action.move_id = current_node.player.get_location();
    int free_index {current_node.id + 1};
    growTurnGraphfromNode (turn_graph, current_node, free_index);
    return turn_graph;
}

// ! WILL NOT WORK
void GameManager::growTurnGraphfromNode (TurnGraph &turn_graph, TurnNode &turn_node, int &free_index)
{
    std::cout << "Entering growTurnGraphfromNode()..." << std::endl;
    PossibleMoves move_candidates {listMovesfromNodeConstrained(turn_node)};
    // ! need to add repair/movement options for the current node! maybe have two searches, one for movement and one for repair

    std::cout << "\tSize of move_candidates from node " << turn_node.id << ": " << move_candidates.size() << std::endl;
    if (move_candidates.size() > 0) {
        for (std::size_t i = 0; i < move_candidates.size(); i++) {
            
            // - fill out child
            int current_location {move_candidates[i].move_id};
            if (current_location == -1) {
                current_location = turn_node.location;
            }
            TurnNode child {free_index, turn_node.id, current_location, turn_node.player, turn_node.repair_board};
            child.action = move_candidates[i];

            // - increment id
            free_index++;

            // - add child id to parent
            turn_node.children.push_back(child.id);
            std::cout << "\t\t\tChild " << child.id << " added to parent " << child.parent << " with action (move: " << child.action.move_id << ", repair: " << child.action.repair_id << ")" << std::endl; 

            // - set recursion if not at terminal node based on move_candidates
            if (move_candidates[i].move_id != -1) {
                child.player.remaining_movement -= 1;
                std::cout << "Child TurnNode has " << child.player.remaining_movement << " remaining movement, its parent has " << turn_node.player.remaining_movement << std::endl;
            } else {
                child.player.remaining_coverage -= 1;
                child.repair_board.at(move_candidates[i].repair_id).covered = true;
                std::cout << "Child TurnNode has " << child.player.remaining_coverage << " remaining coverage, its parent has " << turn_node.player.remaining_coverage << std::endl;
            }

            std::cout << "\t\tRegressing deeper into tree for node " << child.id << std::endl;
            growTurnGraphfromNode (turn_graph, child, free_index);
        }
        std::cout << "Trying to enter node with id " << turn_node.id << " into graph" << std::endl;
        turn_graph.insert({turn_node.id, turn_node});
    }
}

PossibleMoves GameManager::listMovesfromNodeConstrained (TurnNode &turn_node)
{
    PossibleMoves candidates;

    if (turn_node.player.remaining_movement > 0) {
        std::vector<int> movement_edges;
        switch (turn_node.player.get_type()) {
            case 0: // drone
            {
                movement_edges = board_.movement_spaces.at(turn_node.location).drone_edges;
                break;
            }
            case 1: // quadruped
            {
                movement_edges = board_.movement_spaces.at(turn_node.location).quadruped_edges;
                break;
            }
            case 2: // gantry
            {
                movement_edges = board_.movement_spaces.at(turn_node.location).gantry_edges;
                break;
            }
        }
        for (int i = 0; i < movement_edges.size(); i++) {
            Action action;
            action.move_id = movement_edges[i];
            candidates.push_back(action);
        }
    }

    if (turn_node.player.remaining_coverage > 0) {
        // TODO make this reflect only uncovered ones
        std::vector<int> repair_primitives = board_.movement_spaces.at(turn_node.location).repair_edges;
        std::vector<int> repair_edges;
        for (std::size_t i = 0; i < repair_primitives.size(); i++) {
            if (turn_node.repair_board.at(repair_primitives[i]).covered == false) {
                repair_edges.push_back(repair_primitives[i]);
            }
        }
        for (int i = 0; i < repair_edges.size(); i++) {
            Action action;
            action.repair_id = repair_edges[i];
            candidates.push_back(action);
        }
    }
    // std::cout << "At location " << turn_node.location << ", robot has remaining movement of " << turn_node.player.remaining_movement << ", remaining coverage of " << turn_node.player.remaining_coverage << " and size of available moves is " << candidates.size() << "." << std::endl;

    return candidates;
}


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
    return false;
}

bool GameManager::sufficientBattery ()
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
