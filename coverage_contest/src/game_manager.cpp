#include "coverage_contest/game_manager.h"

GameManager::GameManager (bool log_level)
{
    log_level_ = log_level;
}

GameManager::GameManager (Board board, agents::Party party, int player_turn, bool log_level)
{
    board_ = board;
    party_ = party;
    player_turn_ = player_turn;
    total_turns_ = 0;
    log_level_ = log_level;
}

void GameManager::instantiateBoard (const std::string &move_dir, const float &move_discretization, const std::string &repair_dir, const float &repair_discretization)
{
    sensor_msgs::PointCloud2 move_map;
    board_utils::loadCloudasMsg(move_dir, move_map);
    sensor_msgs::PointCloud2 repair_map;
    board_utils::loadCloudasMsg(repair_dir, repair_map);
    board_ = board_utils::generateBoard(move_map, move_discretization, repair_map, repair_discretization);
}

void GameManager::instantiateBoard (const std::string &move_dir, const float &move_discretization, sensor_msgs::PointCloud2 &move_board, const std::string &repair_dir, const float &repair_discretization, sensor_msgs::PointCloud2 &repair_board)
{
    sensor_msgs::PointCloud2 move_map;
    board_utils::loadCloudasMsg(move_dir, move_map);
    sensor_msgs::PointCloud2 repair_map;
    board_utils::loadCloudasMsg(repair_dir, repair_map);
    board_ = board_utils::generateBoard(move_map, move_discretization, repair_map, repair_discretization);
    board_utils::buildColoredClouds(board_, move_board, repair_board);
}

void GameManager::instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries, int starting_position)
{
    party_ = agents::instantiatePlayers(num_drones, num_quadrupeds, num_gantries);
    player_turn_ = 0;
    total_turns_ = 0;
    if (starting_position < 0) {
        // - enumerate possible options
        std::vector<int> viable_starting_locations;
        MoveBoard::iterator it;
        for (it = board_.movement_spaces.begin(); it != board_.movement_spaces.end(); it++) {
            viable_starting_locations.push_back(it->first);
        }
        // - randomly assign one to each of them
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<> distr(0, viable_starting_locations.size() - 1);
        for (std::size_t i = 0; i < party_.playing_order.size(); i++) {
            party_.players.at(party_.playing_order[i]).update_location(viable_starting_locations[distr(rng)]);
        }
    } else {
        for (std::size_t i = 0; i < party_.playing_order.size(); i++) {
            party_.players.at(party_.playing_order[i]).update_location(starting_position);
        }
    }
    if (log_level_) {
        std::cout << "[Manager]\n------------------------------------------------------------\nStarting Party Turn " << total_turns_ << "\n------------------------------------------------------------" << std::endl;
        std::cout << "[Manager] " << party_.playing_order[player_turn_] << " is starting their turn." << std::endl;
    }
}

void GameManager::generateTurnOrder ()
{
    agents::randomShufflePlayingOrder (party_);
    player_turn_ = 0;
}

std::string GameManager::startNext () 
{
    // - reset turn values
    party_.players.at(playingNow()).reset_remaining_movement();
    party_.players.at(playingNow()).reset_remaining_coverage();

    // - increment turn
    player_turn_++;
    if (player_turn_ > (party_.playing_order.size() - 1)) {
        player_turn_ = 0;
        total_turns_++;
        if (log_level_) {
            std::cout << "[Manager]\n------------------------------------------------------------\nStarting Party Turn " << total_turns_ << "\n------------------------------------------------------------" << std::endl;
        }
    }
    if (log_level_) {
        std::cout << "[Manager] " << party_.playing_order[player_turn_] << " is starting their turn." << std::endl;
        std::cout << "\tRemaining battery: " << party_.players.at(party_.playing_order[player_turn_]).remaining_battery << ", Current score: " << party_.players.at(party_.playing_order[player_turn_]).get_score() << std::endl;
    }
    return party_.playing_order[player_turn_];
}

std::string GameManager::playingNow () 
{
    return party_.playing_order[player_turn_];
}

void GameManager::playRandomGame ()
{
    while (!isOver()) {
        takeRandomTurn();
    }
    std::vector<std::string> winners {determineWinners()};
    if (log_level_) {
        std::cout << "[Manager]\n------------------------------------------------------------\nGame has reached terminal state after " << total_turns_  << " turns!" << "\n------------------------------------------------------------" << std::endl;
        std::cout << "\tWinner(s):";
        for (std::string winner : winners) {
            std::cout << winner << " (" << party_.players.at(winner).get_score() << "),";
        }
        std::cout << std::endl;
    }
}

void GameManager::playToDepth (const int &depth)
{
    int num_turns {depth * static_cast<int>(party_.playing_order.size())};
    for (std::size_t i = 0; i < num_turns; i++) {
        if (isOver()) {
            break;
        }
        takeRandomTurn();
    }
    std::vector<std::string> winners {determineWinners()};
    if (log_level_) {
        std::cout << "[Manager] Game has reached recursed to set depth of " << depth  << " turns!" << std::endl;
        std::cout << "\tWinner(s):";
        for (std::string winner : winners) {
            std::cout << winner << " (" << party_.players.at(winner).get_score() << "),";
        }
        std::cout << std::endl;
    }
}

void GameManager::takeRandomTurn ()
{
    // - enforce battery limitations
    if (hasSufficientBattery()) {
        // - sample and play a random turn
        const int num_candidates {10};
        playRandomMove(generateRandomTurns(num_candidates));
    }

    // - reset turn-based decaying values
    party_.players.at(playingNow()).reset_remaining_movement();
    party_.players.at(playingNow()).reset_remaining_coverage();
    if (log_level_) {
        std::cout << "\tPlayer update:" << std::endl;
        std::cout << "\t\tid: " << party_.players.at(playingNow()).get_id() << std::endl;
        std::cout << "\t\tlocation: " << party_.players.at(playingNow()).get_location() << std::endl;
        std::cout << "\t\tscore: " << party_.players.at(playingNow()).get_score() << std::endl;
        std::cout << "\t\tbattery: " << party_.players.at(playingNow()).remaining_battery << std::endl;
        std::cout << "\t\tcharge: " << party_.players.at(playingNow()).remaining_charge_time << std::endl;
    }
    // - start the next player's turn
    startNext();
}

void GameManager::playRandomMove (TurnOptions candidates)
{
    if (candidates.size() > 0) {
        // - randomly pick a possilbe turn
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist(0, candidates.size() - 1);
        playSequence (candidates[dist(rng)]);
    }
}

void GameManager::playSequence (TurnSequence &sequence)
{
    // - play the sequence, updating the global board and party accordingly
    while (!sequence.empty()) {
        Action action {sequence.front()};
        if (action.move_id != -1) {
            // - handle movements
            party_.players.at(playingNow()).update_location(action.move_id);
            if (log_level_) {std::cout << "\t\tRobot moved to move node " << action.move_id << std::endl;}
        } else {
            // - handle repairs and add to score
            board_.repair_spaces.at(action.repair_id).covered = true;
            party_.players.at(playingNow()).update_score(1);
            if (log_level_) {std::cout << "\t\tRobot repaired repair node " << action.repair_id << std::endl;}
        }
        sequence.pop();
    }
}

bool GameManager::isOver ()
{
    RepairBoard::iterator it;
    for (it = board_.repair_spaces.begin(); it != board_.repair_spaces.end(); it++) {
        if (it->second.covered == false) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> GameManager::determineWinners ()
{
    std::vector<std::string> winners;
    std::map<std::string, agents::Robot>::iterator it;
    for (it = party_.players.begin(); it != party_.players.end(); it++) {
        if (winners.size() < 1) {
            winners.push_back(it->first);
        } else if (it->second.get_score() > party_.players.at(winners[0]).get_score()) {
            winners.clear();
            winners.push_back(it->first);
        } else if (it->second.get_score() == party_.players.at(winners[0]).get_score()) {
            winners.push_back(it->first);
        }
    }
    return winners;
}

TurnOptions GameManager::generateRandomTurns (const int num_moves)
{
    TurnOptions possible_turns {};
    for (std::size_t i = 0; i < num_moves; i++) {
        possible_turns.push_back(generateRandomTurn());
    }
    return possible_turns;
}

TurnSequence GameManager::generateRandomTurn ()
{
    TurnSequence possible_turn;

    std::random_device dev;
    std::mt19937 rng(dev());

    // - for current player, get starting state
    int state {party_.players.at(playingNow()).get_location()};
    bool reached_terminal_state {false};

    // - add to move until turn reaches a terminal state
    agents::Robot player {party_.players.at(playingNow())};
    RepairBoard board {board_.repair_spaces};

    while (!reached_terminal_state) {
        MoveOptions candidates {listMovesfromNodeConstrained(player, board)};

        // - return if we are at a terminal state
        if (candidates.size() < 1) {
            reached_terminal_state = true;
        } else {
            // - randomly sample one of these
            std::uniform_int_distribution<std::mt19937::result_type> dist(0, candidates.size() - 1);
            Action move = candidates[dist(rng)];

            // - add it to the TurnSequence and update state variable
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

MoveOptions GameManager::listMovesfromNodeConstrained (agents::Robot &player, RepairBoard &board)
{
    MoveOptions candidates;

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
        
        if (party_.players.at(playingNow()).get_type() == 2) { // if gantry
            if (board_.movement_spaces.at(player.get_location()).neighbors.z_pos != -1) {
                std::vector<int> high_repair_primitives {board_.movement_spaces.at(board_.movement_spaces.at(player.get_location()).neighbors.z_pos).repair_edges};
                repair_primitives.insert(repair_primitives.end(), high_repair_primitives.begin(), high_repair_primitives.end());
            }
        }
        
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

MoveOptions GameManager::listMovesfromNode (const int &index)
{
    MoveOptions candidates;

    // - create a move for all movement options and repair options
    std::vector<int> movement_edges;
    std::vector<int> repair_edges {board_.movement_spaces.at(index).repair_edges};

    if (party_.players.at(playingNow()).get_type() == 2) { // if gantry
        if (board_.movement_spaces.at(index).neighbors.z_pos != -1) {
            // - add these edges
            std::vector<int> high_repair_edges {board_.movement_spaces.at(board_.movement_spaces.at(index).neighbors.z_pos).repair_edges};
            repair_edges.insert(repair_edges.end(), high_repair_edges.begin(), high_repair_edges.end());
        }
    }

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

MoveOptions GameManager::listMovesfromNode ()
{
    return listMovesfromNode (party_.players.at(playingNow()).get_location());
}

bool GameManager::hasSufficientBattery ()
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

void GameManager::printMovesfromState (int state)
{
    std::cout << "Move candidates at state " << state << ":" << std::endl;
    MoveOptions candidates {listMovesfromNode(state)};
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
    TurnOptions candidates {generateRandomTurns(num)};

    for (std::size_t i = 0; i < candidates.size(); i++) {
        std::cout << "Turn option " << static_cast<int>(i) << ": " << std::endl;
        while (!candidates[i].empty()) {
            Action action {candidates[i].front()};

            std::cout << "\tAction: (move: " << action.move_id << ", repair: " << action.repair_id << ")" << std::endl; 

            candidates[i].pop();
        }
    }
}

void GameManager::printSequence (TurnSequence sequence)
{
    while(!sequence.empty()) {
        Action action {sequence.front()};
        std::cout << "\tAction: (move: " << action.move_id << ", repair: " << action.repair_id << ")" << std::endl; 
        sequence.pop();
    }
}

void GameManager::printScoreboard ()
{
    std::cout << "\tScores: " << std::endl;
    for (std::size_t i = 0; i < party_.playing_order.size(); i++) {
        std::cout << "\t\t" << party_.players.at(party_.playing_order[i]).get_id() << ": " << party_.players.at(party_.playing_order[i]).get_score() << std::endl;
    }
}
