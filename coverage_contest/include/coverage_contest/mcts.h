#ifndef MONTE_CARLO_TREE_SEARCH
#define MONTE_CARLO_TREE_SEARCH

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include "board.hpp"
#include "agents.hpp"
#include <chrono>


struct ActiveBoard
{
    agents::Party party;
};
using Children = std::map<int, ActiveBoard>;

typedef struct Leaf {
    bool is_root = false;
    ActiveBoard active_board;
    Children children;
    float total_value;
    int num_episodes;
    struct Leaf* parent;
} Leaf;

class MCTS
{
public:
    MCTS();
private:
    void constructTreeSearch (Board &board, std::chrono::duration<float> duration);
    Leaf upperConfidenceStrategy (Leaf &node);
    Leaf findLeaf (Leaf &node);
    int simulate (Leaf &node);
    void backpropagate (Leaf &node, float result);
};

#endif // MONTE_CARLO_TREE_SEARCH
