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

void GameManager::testRandomTurns (int num)
{
    PossibleTurns candidates {generateRandomTurns(num)};

    for (std::size_t i = 0; i < candidates.size(); i++) {
        std::cout << "Turn option " << static_cast<int>(i) << ": " << std::endl;
        while (!candidates[i].empty()) {
            Action action {candidates[i].front()};

            std::cout << "\tAction: (move: " << action.move_id << ", repair: " << action.repair_id << ")" << std::endl; 

            candidates[i].pop();
        }
    }
}

PossibleTurns GameManager::generateRandomTurns (const int num_moves)
{
    PossibleTurns possible_turns;
    for (std::size_t i = 0; i < num_moves; i++) {
        possible_turns.push_back(generateRandomTurn());
    }
    return possible_turns;
}

PossibleTurn GameManager::generateRandomTurn ()
{
    PossibleTurn possible_turn;

    std::random_device dev;
    std::mt19937 rng(dev());

    // - for current player, get starting state
    int state {party_.players.at(playingNow()).get_location()};
    bool reached_terminal_state {false};

    // - add to move until turn reaches a terminal state
    agents::Robot player {party_.players.at(playingNow())};
    RepairBoard board {board_.repair_spaces};

    while (!reached_terminal_state) {
        PossibleMoves candidates {listMovesfromNodeConstrained(player, board)};

        // - return if we are at a terminal state
        if (candidates.size() < 1) {
            reached_terminal_state = true;
        } else {
            // - randomly sample one of these
            std::uniform_int_distribution<std::mt19937::result_type> dist(0, candidates.size() - 1);
            Action move = candidates[dist(rng)];

            // - add it to the PossibleTurn and update state variable
            possible_turn.push(move);

            // - simulate robot taking that action
            if (move.move_id != -1) {
                player.remaining_movement -= 1;
                player.update_location(move.move_id);
            } else {
                player.remaining_coverage -=1;
                board.at(move.repair_id).covered = true;
            }
        }
    }

    return possible_turn;
}


PossibleMoves GameManager::listMovesfromNodeConstrained (agents::Robot &player, RepairBoard &board)
{
    PossibleMoves candidates;

    if ((player.remaining_movement > 0) && (player.remaining_coverage == player.get_max_turn_coverage())) {
        std::vector<int> movement_edges;
        switch (player.get_type()) {
            case 0: // drone
            {
                movement_edges = board_.movement_spaces.at(player.get_location()).drone_edges;
                break;
            }
            case 1: // quadruped
            {
                movement_edges = board_.movement_spaces.at(player.get_location()).quadruped_edges;
                break;
            }
            case 2: // gantry
            {
                movement_edges = board_.movement_spaces.at(player.get_location()).gantry_edges;
                break;
            }
        }
        for (int i = 0; i < movement_edges.size(); i++) {
            Action action;
            action.move_id = movement_edges[i];
            candidates.push_back(action);
        }
    }

    if (player.remaining_coverage > 0) {
        std::vector<int> repair_primitives = board_.movement_spaces.at(player.get_location()).repair_edges;
        std::vector<int> repair_edges;
        for (std::size_t i = 0; i < repair_primitives.size(); i++) {
            if (board.at(repair_primitives[i]).covered == false) {
                repair_edges.push_back(repair_primitives[i]);
            }
        }
        for (int i = 0; i < repair_edges.size(); i++) {
            Action action;
            action.repair_id = repair_edges[i];
            candidates.push_back(action);
        }
    }
    return candidates;
}


void GameManager::playRandomMove ()
{

}

// TODO
// - finish is_over function
// - then make something to play the whole game with each player taking a random turn until someone wins
// - also need a way of actually playing the turns, so a function that will take in the queue and go through it, updating robot score, movement, coverage, etc.





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
