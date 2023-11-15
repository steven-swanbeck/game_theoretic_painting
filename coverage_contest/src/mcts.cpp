#include "coverage_contest/mcts.h"

MCTS::MCTS (Board board, agents::Party party, int player_turn) {
    manager_ = GameManager(board, party, player_turn);
}

TurnSequence MCTS::constructTreeSearch (int num_candidates, std::chrono::duration<float> duration)
{
    // - generate num_candidates moves
    TurnOptions moves {manager_.generateRandomTurns(num_candidates)};

    // - generate leaves for each of these
    Children children;
    for (std::size_t i = 0; i < children.size(); i++) {
        Leaf leaf (manager_.board_, manager_.party_, manager_.player_turn_);
        leaf.turn = moves[i];
        children.push_back(leaf);
    }

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

Leaf MCTS::upperConfidenceStrategy (Children &children)
{
    
}

Leaf MCTS::findLeaf (Children &children)
{
    
}

void MCTS::backpropagate (Leaf &node, float result)
{
    node.episodes++;
    
}
