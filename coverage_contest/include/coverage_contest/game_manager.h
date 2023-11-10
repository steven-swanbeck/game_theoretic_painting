#ifndef GAME_MANAGER_H
#define GAME_MANAGER_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include "board.hpp"
#include "agents.hpp"

// template <class AgentT>
class GameManager
{
public:
    GameManager ();
    void instantiateBoard (const std::string &move_dir, const float &move_discretization, const std::string &repair_dir, const float &repair_discretization);
    void instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries, int starting_position=0);
    void generateTurnOrder ();
    std::string startNext ();
    std::string playingNow ();
    bool sufficientBattery ();
    void takeTurn ();
    void playRandomMove ();
    void simulateRandomGame ();
    void assessValue ();
    bool isOver ();
    PossibleMoves listMovesfromNode ();
    PossibleMoves listMovesfromNode (const int &index);
    PossibleMoves listMovesfromNode (TurnNode &turn_node);
    PossibleMoves listMovesfromNodeConstrained (TurnNode &turn_node);
    TurnGraph createTurnGraph ();
    void growTurnGraphfromNode (TurnGraph &turn_graph, TurnNode &turn_node, int &free_index);
    void growRepairScheduleWithinNode (TurnGraph &turn_graph, TurnNode &turn_node, int &free_index);
    void printMovesfromState (int state);

    // std::vector<int> nextMoves (AgentT agent);
    // template <typename agents::Quadruped> std::vector<int> nextMoves (agents::Quadruped agent);
    // template <typename agents::Gantry> std::vector<int> nextMoves (agents::Gantry agent);

private:
    Board board_;
    agents::Party party_;
    int player_turn_;
    int total_turns_;
};

#endif // GAME_MANAGER_H
