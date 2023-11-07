#ifndef GAME_PLAYERS_H
#define GAME_PLAYERS_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"
#include <algorithm>
#include <random>
#include <any>

namespace agents
{
// . Base class
class Robot
{
public:
    Robot (std::string identifier, bool reach_upper, int max_movement, int max_coverage, int battery_life, int recharge_duration) {
        id = identifier;
        can_reach_upper = reach_upper;
        max_turn_movement = max_movement;
        remaining_movement = max_movement;
        max_battery_life = battery_life;
        remaining_battery = battery_life;
        recharge_time = recharge_duration;
        remaining_charge_time = 0;
        int score = 0;
    }
    // . Info
    int get_max_turn_movement() {
        return max_turn_movement;
    }
    bool get_can_reach_upper () {
        return can_reach_upper;
    }
    int get_max_battery() {
        return max_battery_life;
    }
    int get_recharge_time () {
        return recharge_time;
    }
    int get_max_turn_coverage () {
        return max_turn_coverage;
    }
    // . Reset
    void reset_remaining_movement () {
        remaining_movement = max_turn_movement;
    }
    void reset_remaining_battery () {
        remaining_battery = max_battery_life;
    }
    void reset_remaining_charge_time () {
        remaining_charge_time = 0;
    }
    void reset_remaining_coverage () {
        remaining_charge_time = max_turn_coverage;
    }
    void reset_score () {
        score = 0;
    }
    // . Update
    void update_position (int index) {
        position = index;
    }
    void update_score (int update) {
        score += update;
    }

    std::string id;
    int remaining_movement;
    int remaining_battery;
    int remaining_charge_time;
    int remaining_coverage;

private:
    bool can_reach_upper;
    int max_turn_movement;
    int max_battery_life;
    int recharge_time;
    int max_turn_coverage;
    int position;
    int score;
};

// . Drone, Quadruped, Gantry classes
class Drone : public Robot
{
public:
    Drone(std::string identifier) : Robot(identifier, true, 5, 10, 2, 2) {}
};

class Quadruped : public Robot
{
public:
    Quadruped(std::string identifier) : Robot(identifier, false, 3, 25, 10, 2) {}
};

class Gantry : public Robot
{
public:
    Gantry(std::string identifier) : Robot(identifier, true, 1, 50, 30, 2) {}
};

// . Instantiation function
struct Party
{
    std::map<std::string, Robot> players;
    std::vector<std::string> playing_order;
};

void randomShufflePlayingOrder (agents::Party &party)
{
    std::shuffle(std::begin(party.playing_order), std::end(party.playing_order), std::random_device());
    std::cout << "[Agents] Player turn order randomly shuffled as: \n";
    for (std::size_t i = 0; i < party.playing_order.size(); i++) {
        std::cout << "\t" << static_cast<int>(i) << ": " << party.playing_order[i] << std::endl;;
    }
}

Party instantiatePlayers (const int num_drones, const int num_quadrupeds, const int num_gantries, bool random_shuffle=true)
{
    Party party;
    for (std::size_t i = 0; i < num_drones; i++) {
        std::string agent_name {"drone_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Drone(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    for (std::size_t i = 0; i < num_quadrupeds; i++) {
        std::string agent_name {"quadruped_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Quadruped(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    for (std::size_t i = 0; i < num_gantries; i++) {
        std::string agent_name {"gantry_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Gantry(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    if (random_shuffle) {
        randomShufflePlayingOrder(party);
    }
    return party;
}

}

#endif // GAME_PLAYERS_H
