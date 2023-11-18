#ifndef MONTE_CARLO_TREE_SEARCH
#define MONTE_CARLO_TREE_SEARCH

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "coverage_contest/game_manager.h"
#include "board.hpp"
#include "agents.hpp"
#include <chrono>
#include <thread>

/** @struct PrimalLeaf
 * @brief used as immediate child of root in MCTS formulation; ie. the sequences of actions that can actually be made in a given turn
 * @var PrimalLeaf::game_state
 * 'game_state' copies the board and party details from a higher manager and is used to simulate play
 * @var PrimalLeaf::value
 * 'value' is the total value of the leaf that is accumulated as episodes are played through it
 * @var PrimalLeaf::episodes
 * 'episodes' is the total number of episodes that have been played through the node
 * @var PrimalLeaf::turn
 * 'turn' denotes the sequence of actions used to arrive at the primal leaf's game state from the true state
 */
struct PrimalLeaf {
    GameManager game_state;
    float value {0};
    int episodes {0};
    TurnSequence turn;
    PrimalLeaf(Board board, agents::Party party, int player_turn) : game_state(board, party, player_turn) {}
};

/** @struct GeneticLeaf
 * @brief used as descendent of primal nodes in MCTS formulation; ie. the sequences of actions beyond those that can actually be made in a given turn
 * @var GeneticLeaf::game_state
 * 'game_state' copies the board and party details from a higher manager and is used to simulate play
 * @var GeneticLeaf::value
 * 'value' is the total value of the leaf that is accumulated as episodes are played through it
 * @var GeneticLeaf::episodes
 * 'episodes' is the total number of episodes that have been played through the node
 */
struct GeneticLeaf {
    GameManager game_state;
    float value {0};
    int episodes {0};
    GeneticLeaf(GameManager state) : game_state(state.board_, state.party_, state.player_turn_) {}
};

typedef std::vector<PrimalLeaf> Ancestors;
typedef std::vector<GeneticLeaf> Descendents;

/** @class MCTS
 * @brief randomly generates a tree of sampled possible future game states and uses them to recommend the immediate action of the current game player
 * @var MCTS::manager_
 * member 'manager_' contains the root game state that is copied and mutated by class methods
 * @var MCTS::num_candidates_
 * member 'num_candidates_' is the number of randomly sampled moves used to construct the first 'primal' layer of the tree
 * @var MCTS::search_duration_ms_
 * member 'search_duration_ms_' is the time in ms the MCTS algorithm is allowed to simulate before it returns a move 
 * @var MCTS::search_depth_
 * member 'search_depth_' is the number of party turns into the future the tree search will extend for all given episodes 
 * @var MCTS::c_
 * member 'c_' is used by the upper confidence strategy for leaf selection and parameterizes the explore-exploit behavior dichotomy
 */
class MCTS
{
public:
    MCTS(Board board, agents::Party party, int player_turn, int duration=1000, int candidates=20, int search_depth=5, float uct_c=1.4142136);

    /** Constructs a tree search and plays episodes through it following class parameters to return the most favorable immediate sequence of actions for a player to take
     * @brief randomly samples and simulates play through possible game states
     */
    TurnSequence search ();

private:
    /** Uses game manager to create a set of primal nodes corresponding to possible board states that can be reached this turn
     * @brief generates primal leaves with viable turn sequences
     * @return vector of primal nodes
     */
    Ancestors generateCandidates ();

    /** Using a turn sequence, creates a primal leaf with updated game state
     * @brief uses a turn sequence to propagate the current game state into a possible future state
     * @param turn TurnSequence type, possible turn from current game state used to simulate forward in time
     * @return primal node derived from possible turn sequence
     */
    PrimalLeaf generateCandidate (TurnSequence &turn);

    /** Given a set of primal nodes, chooses one to explore using an explore-exploit paradigm
     * @brief select the 'best' of a set of primal nodes from which to simulate play
     * @param candidates Ancestors type, all existing primal nodes
     * @return index in candidates corresponding to the selected primal node
     */
    int upperConfidenceStrategy (Ancestors &candidates);

    /** Given a set of primal nodes, choose one 
     * @brief select the 'best' of a set of primal nodes from which to simulate play
     * @param candidates Ancestors type, all existing primal nodes
     * @return index in candidates corresponding to the selected primal node
     */
    int findLeaf (Ancestors &candidates);

    /** Simulates turns to a predefined tree depth for all players
     * @brief simulates gameplay from a given game state
     * @param node GeneticLeaf type, possible future state given an immediate turn sequence
     */
    void simulate (GeneticLeaf &node);

    /** Backpropagates information upon completion of a simulation
     * @brief backprogagates cumulative value and episode count from a descendent to its ancestor
     * @param ancestor PrimalLead type, values are updated using simulation results of descendent that descended from it
     * @param descendent GeneticLeaf type, used to update parameters of parent
     */
    void backpropagate (PrimalLeaf &ancestor, GeneticLeaf &descendent);

    /** Calculates the value of all players at a given game state
     * @brief Calculates the value of all players at a given game state
     * @param state GameManager type, state from which total value is calculated
     * @return cumulative value of all players at a given game state
     */
    float calculateCumulativeValue (GameManager &state);

    /** Calculates the average value of players at a given game state
     * @brief Calculates the average value of players at a given game state
     * @param state GameManager type, state from which total value is calculated
     * @return average value of players at a given game state
     */
    float calculateAverageValue (GameManager &state);

    /** Determines the primal leaf with the largest total value
     * @brief Determines the primal leaf with the largest total value
     * @param candidates Ancestor type, vector of primal nodes
     * @return index of the largest-valued primal node in the input vector
     */
    int maxValuedLeaf (Ancestors &candidates);

    // Testing functions
    void printCandidates (Ancestors &candidates);

    GameManager manager_;
    int num_candidates_;
    int search_duration_ms_;
    int search_depth_;
    float c_;
};

#endif // MONTE_CARLO_TREE_SEARCH
