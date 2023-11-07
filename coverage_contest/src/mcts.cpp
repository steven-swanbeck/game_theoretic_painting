#include "coverage_contest/mcts.h"

MCTS::MCTS () {

}

void MCTS::constructTreeSearch (Board &board, std::chrono::duration<float> duration)
{
    
}

Leaf MCTS::upperConfidenceStrategy (Leaf &node)
{
    
}

Leaf MCTS::findLeaf (Leaf &node)
{
    
}

int MCTS::simulate (Leaf &node)
{
    
}

void MCTS::backpropagate (Leaf &node, float result)
{
    node.num_episodes++;
    node.total_value += result;

    if (node.is_root) {
        return;
    }
    backpropagate (*(node.parent), result);
}
