#ifndef MONTE_CARLO_TREE_SEARCH
#define MONTE_CARLO_TREE_SEARCH

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "coverage_contest/game_manager.h"
#include "board.hpp"
#include "agents.hpp"
#include <chrono>


struct DynamicBoard
{
    agents::Party party;
    RepairBoard repair_board;
};

struct PrimalLeaf {
    GameManager game_state;
    float value {0};
    int episodes {0};
    TurnSequence turn;
    PrimalLeaf(Board board, agents::Party party, int player_turn) : game_state(board, party, player_turn) {}
};

struct GeneticLeaf {
    GameManager game_state;
    float value {0};
    int episodes {0};
    GeneticLeaf(Board board, agents::Party party, int player_turn) : game_state(board, party, player_turn) {}
};

typedef std::vector<PrimalLeaf> Ancestors;
typedef std::vector<GeneticLeaf> Children;

// TODO
// - generate set of random turns from the current state
// - enact each of these turns on a separate game manager object
// - using export-exploit strategy, randomly simulate the game until a certain depth or until completion
// - repeat for a given amount of time, then take the option with the highest value
// - give reward for relative performance over/under expectation
// + for this, calculate the max number of spaces to be repaired, divide by the number of players, and use positive or negative difference to update score
// - select the leaf (turn) with the highest value and enact it, then repeat for all other players

class MCTS
{
public:
    MCTS(Board board, agents::Party party, int player_turn);
private:
    // TurnSequence constructTreeSearch (int num_candidates, std::chrono::duration<float> duration);
    TurnSequence constructTreeSearch (int num_candidates, std::chrono::duration<float> duration);
    Ancestors generateCandidates (int num_candidates);
    PrimalLeaf generateCandidate (TurnSequence &turn);
    PrimalLeaf upperConfidenceStrategy (Ancestors &candidates, float c=1.4142136);
    PrimalLeaf findLeaf (Ancestors &candidates);
    void backpropagate (PrimalLeaf &ancestor, GeneticLeaf &child);
    float calculateCumulativeValue (GameManager &state);
    float calculateAverageValue (GameManager &state);

    GameManager manager_;
};

#endif // MONTE_CARLO_TREE_SEARCH
