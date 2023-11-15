#include "coverage_contest/mcts.h"

MCTS::MCTS (Board board, agents::Party party, int player_turn) {
    manager_ = GameManager(board, party, player_turn);
}

// TODO
TurnSequence MCTS::constructTreeSearch (int num_candidates, std::chrono::duration<float> duration)
{
    Ancestors candidates {generateCandidates(num_candidates)};

    // - for duration, create game manager from that state and play it all the way through
    // auto finish = std::chrono::system_clock::now() + std::chrono::seconds(1);
    auto finish = std::chrono::system_clock::now() + duration;
    do {
        // - choose leaf to visit using UCT

        // - copy game manager

        // - compute possible score here? or maybe just sum score of all players at end?

        // - simulate random game

        // - compute score

        // - backpropagate score and update num_episodes

    } while (std::chrono::system_clock::now() < finish);

    // - once time has expired, use UCT to pick the best enumerated move

    // - return it and execute
}

// DONE
Ancestors MCTS::generateCandidates (int num_candidates)
{
    // - generate num_candidates moves
    TurnOptions options {manager_.generateRandomTurns(num_candidates)};

    // - generate leaves for each of these
    Ancestors candidates;
    for (std::size_t i = 0; i < options.size(); i++) {
        candidates.push_back(generateCandidate(options[i]));
    }
}

// WIP need to make sure this fills everything out as desired (without mutating anything)
PrimalLeaf MCTS::generateCandidate (TurnSequence &turn)
{
    // - create leaf
    PrimalLeaf candidate (manager_.board_, manager_.party_, manager_.player_turn_);
    
    // - assign and play turn
    candidate.turn = turn; // ! be aware this may still be mutated unintentionally
    candidate.game_state.playSequence(turn);

    // - calculate value (from turn)
    candidate.value = candidate.game_state.party_.players.at(candidate.game_state.party_.playing_order[candidate.game_state.player_turn_]).get_score();

    return candidate;
}

// DONE
PrimalLeaf MCTS::upperConfidenceStrategy (Ancestors &candidates, float c)
{
    // - already have vector of candidates with scores and episodes
    int total_episodes {};
    for (std::size_t i = 0; i < candidates.size(); i++) {
        total_episodes += candidates[i].episodes;
    }

    // - accumulate values for each child
    std::vector<double> values (candidates.size());
    for (std::size_t i = 0; i < candidates.size(); i++) {
        double explore {c * sqrt(log(total_episodes) / candidates[i].episodes)};
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
    return candidates[max_value_index];
}

// DONE
PrimalLeaf MCTS::findLeaf (Ancestors &candidates)
{
    return upperConfidenceStrategy(candidates);
}

// TODO
void MCTS::backpropagate (PrimalLeaf &ancestor, GeneticLeaf &child)
{
    ancestor.episodes++;

    float value_difference {child.value - ancestor.value};
    float average_value_difference {calculateAverageValue(child.game_state) - calculateAverageValue(ancestor.game_state)};


    // TODO calculate cumulative change in reward for all players, calculate average reward each player could get, assign value for performance relative to this value  
}

// DONE
float MCTS::calculateCumulativeValue (GameManager &state)
{
    // - loop through all players, sum their scores
    float total_value {};
    // for (std::size_t i = 0; i < state.party_.playing_order.size(); i++) {
    for (std::string player_id : state.party_.playing_order) {
        total_value += state.party_.players.at(player_id).get_score();
    }
    return total_value;
}

// DONE
float MCTS::calculateAverageValue (GameManager &state)
{
    return calculateCumulativeValue(state) / state.party_.playing_order.size();
}
