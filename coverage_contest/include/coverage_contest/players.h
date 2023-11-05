#ifndef GAME_PLAYERS_H
#define GAME_PLAYERS_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"

// TODO construct a base class for player that every different player will inherit from
// - movement for a single turn
// - max reach
// - battery or something similar (?)
// - efficiency somehow?

// . Base class
class Player
{
public:
    int get_max_movement() {
        return max_turn_movement;
    }

protected:
    int max_reach;
    int max_turn_movement;
    int remaining_movement;

};

// . Drone, Quadruped, Gantry classes


#endif // GAME_PLAYERS_H
