#ifndef GAME_PLAYERS_H
#define GAME_PLAYERS_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"

// TODO construct a base class for player that every different player will inherit from
// - movement for a single turn
// - max reach
// - battery or something similar (?)
// - efficiency somehow?

namespace agents
{
// . Base class
class Robot
{
public:
    Robot (bool can_reach_upper, int max_movement, int battery_life, int recharge_duration) {
        reach_upper = can_reach_upper;
        max_turn_movement = max_movement;
        remaining_movement = max_movement;
        max_battery_life = battery_life;
        remaining_battery = battery_life;
        recharge_time = recharge_duration;
        remaining_charge_time = 0;
    }
    int get_max_movement() {
        return max_turn_movement;
    }
    bool can_reach_upper () {
        return reach_upper;
    }
    int get_max_battery() {
        return max_battery_life;
    }
    int get_charge_time () {
        return recharge_time;
    }

    void reset_remaining_movement () {
        remaining_movement = max_turn_movement;
    }
    void reset_remaining_battery () {
        remaining_battery = max_battery_life;
    }
    void reset_remaining_charge_time () {
        remaining_charge_time = 0;
    }

    int remaining_movement;
    int remaining_battery;
    int remaining_charge_time;

private:
    bool reach_upper;
    int max_turn_movement;
    int max_battery_life;
    int recharge_time;
};

// . Drone, Quadruped, Gantry classes
class Drone : public Robot
{
public:
    std::string id;
    Drone(std::string identifier) : Robot(true, 5, 2, 2) {id = identifier;}
};

class Quadruped : public Robot
{
public:
    std::string id;
    Quadruped(std::string identifier) : Robot(false, 3, 10, 2) {id = identifier;}
};

class Gantry : public Robot
{
public:
    std::string id;
    Gantry(std::string identifier) : Robot(true, 1, 30, 2) {id = identifier;}
};

// . Instantiation function
struct Players
{
    std::vector<Drone> drones;
    std::vector<Quadruped> quadrupeds;
    std::vector<Gantry> gantries;
};

Players instantiatePlayers (const int num_drones, const int num_quadrupeds, const int num_gantries)
{
    Players players;
    for (std::size_t i = 0; i < num_drones; i++) {
        players.drones.push_back(Drone("drone_" + std::to_string(static_cast<int>(i))));
    }
    for (std::size_t i = 0; i < num_quadrupeds; i++) {
        players.quadrupeds.push_back(Quadruped("quadruped_" + std::to_string(static_cast<int>(i))));
    }
    for (std::size_t i = 0; i < num_gantries; i++) {
        players.gantries.push_back(Gantry("gantry_" + std::to_string(static_cast<int>(i))));
    }
    return players;
}
}

#endif // GAME_PLAYERS_H
