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
    void instantiateBoard (const std::string &dir, const float &discretization);
    void instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries);
    void generateTurnOrder ();
    std::string upNext ();
    void playRandomMove ();
    void simulateRandomGame ();
    void assessValue ();
    bool isOver ();

    // std::vector<int> nextMoves (AgentT agent);
    // template <typename agents::Quadruped> std::vector<int> nextMoves (agents::Quadruped agent);
    // template <typename agents::Gantry> std::vector<int> nextMoves (agents::Gantry agent);

private:
    Board board_;
    agents::Party party_;
    int player_turn_;
};

#endif // GAME_MANAGER_H
