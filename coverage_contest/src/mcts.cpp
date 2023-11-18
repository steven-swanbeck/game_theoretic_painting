#include "coverage_contest/mcts.h"

MCTS::MCTS (Board board, agents::Party party, int player_turn, int duration, int candidates, int search_depth, float uct_c) {
    manager_ = GameManager(board, party, player_turn);
    search_duration_ms_ = duration;
    num_candidates_ = candidates;
    search_depth_ = search_depth;
    c_ = uct_c;
}

TurnSequence MCTS::search ()
{
    Ancestors candidates {generateCandidates()};

    // int it {};
    std::chrono::milliseconds durationToRun(search_duration_ms_);
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - startTime < durationToRun) {
        // auto elapsedTime = std::chrono::steady_clock::now() - startTime;
        // std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count() << " ms" << std::endl;
        // std::cout << "MCTS Iteration: " << it << std::endl;
        // - choose leaf to visit
        int ancestor_id {findLeaf(candidates)};

        // - construct a child
        GeneticLeaf child {GeneticLeaf(candidates[ancestor_id].game_state)};

        // - simulate random game
        simulate(child);

        // - backpropagate score and update num_episodes
        backpropagate(candidates[ancestor_id], child);

        // it++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // - once time has expired, pick the best sampled move
    // int apex_id {findLeaf(candidates)};
    int apex_id {maxValuedLeaf(candidates)};

    // printCandidates(candidates);

    // - return it and execute
    return candidates[apex_id].turn;
}

void MCTS::printCandidates (Ancestors &candidates) 
{
    std::cout << "Candidates: " << std::endl;
    for (int i = 0; i < candidates.size(); i++) {
        std::cout << "\t" << candidates[i].value << ", " << candidates[i].episodes << std::endl;
        // candidates[i].game_state.printSequence(candidates[i].turn);
    }
}

void MCTS::simulate (GeneticLeaf &node)
{
    // std::cout << "Player id in: " << node.game_state.party_.players.at(node.game_state.party_.playing_order[node.game_state.player_turn_]).get_id() << ", " << node.game_state.total_turns_ << std::endl;
    
    node.game_state.playToDepth(search_depth_);
    node.value = node.game_state.party_.players.at(node.game_state.party_.playing_order[node.game_state.player_turn_]).get_score();

    // std::cout << "Player id out: " << node.game_state.party_.players.at(node.game_state.party_.playing_order[node.game_state.player_turn_]).get_id() << ", " << node.game_state.total_turns_ << std::endl;
}

int MCTS::maxValuedLeaf (Ancestors &candidates)
{
    float max_value_index {0};
    for (std::size_t i = 1; i < candidates.size(); i++) {
        if (candidates[i].value > candidates[max_value_index].value) {
            max_value_index = static_cast<int>(i);
        }
    }
    return max_value_index;
}

Ancestors MCTS::generateCandidates ()
{
    // - generate num_candidates moves
    TurnOptions options {manager_.generateRandomTurns(num_candidates_)};

    // - generate leaves for each of these
    Ancestors candidates;
    for (std::size_t i = 0; i < options.size(); i++) {
        candidates.push_back(generateCandidate(options[i]));
    }

    return candidates;
}

PrimalLeaf MCTS::generateCandidate (TurnSequence &turn)
{
    // - create leaf
    PrimalLeaf candidate (manager_.board_, manager_.party_, manager_.player_turn_);
    
    // - assign and play turn
    candidate.episodes = 1;
    candidate.turn = turn; // ! be aware this may still be mutated unintentionally
    candidate.game_state.playSequence(turn);

    // - calculate value (from turn)
    candidate.value = candidate.game_state.party_.players.at(candidate.game_state.party_.playing_order[candidate.game_state.player_turn_]).get_score();

    return candidate;
}

int MCTS::upperConfidenceStrategy (Ancestors &candidates)
{
    // - already have vector of candidates with scores and episodes
    int total_episodes {};
    for (std::size_t i = 0; i < candidates.size(); i++) {
        total_episodes += candidates[i].episodes;
    }

    // - accumulate values for each child
    std::vector<double> values (candidates.size());
    for (std::size_t i = 0; i < candidates.size(); i++) {
        double explore {c_ * sqrt(log(total_episodes) / candidates[i].episodes)};
        double exploit {candidates[i].value / candidates[i].episodes};
        values[i] = explore + exploit;
    }

    // - pick largest
    float max_value_index {0};
    for (std::size_t i = 1; i < values.size(); i++) {
        if (values[i] > values[max_value_index]) {
            max_value_index = static_cast<int>(i);
        }
    }

    // - return the corresponding leaf
    // return candidates[max_value_index];
    return max_value_index;
}

int MCTS::findLeaf (Ancestors &candidates)
{
    return upperConfidenceStrategy(candidates);
}

void MCTS::backpropagate (PrimalLeaf &ancestor, GeneticLeaf &child)
{
    ancestor.episodes++;
    ancestor.value += child.value;

    // float value_difference {child.value - ancestor.value};
    // float average_value_difference {calculateAverageValue(child.game_state) - calculateAverageValue(ancestor.game_state)};
    // TODO calculate cumulative change in reward for all players, calculate average reward each player could get, assign value for performance relative to this value  
}

float MCTS::calculateCumulativeValue (GameManager &state)
{
    // - loop through all players, sum their scores
    float total_value {};
    for (std::string player_id : state.party_.playing_order) {
        total_value += state.party_.players.at(player_id).get_score();
    }
    return total_value;
}

float MCTS::calculateAverageValue (GameManager &state)
{
    return calculateCumulativeValue(state) / state.party_.playing_order.size();
}
