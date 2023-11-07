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
    Robot (std::string handle, int type_enum, bool reach_upper, int max_movement, int max_coverage, int battery_life, int recharge_duration) {
        id = handle;
        type = type_enum;
        can_reach_upper = reach_upper;
        max_turn_movement = max_movement;
        max_turn_coverage = max_coverage;
        max_battery_life = battery_life;
        remaining_movement = max_movement;
        remaining_coverage = max_coverage;
        remaining_battery = battery_life;
        recharge_time = recharge_duration;
        remaining_charge_time = recharge_time;
        score = 0;
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
    std::string get_id () {
        return id;
    }
    int get_type () {
        return type;
    }
    int get_location () {
        return location;
    }
    int get_score () {
        return score;
    }
    // . Reset
    void reset_remaining_movement () {
        remaining_movement = max_turn_movement;
        std::cout << "\t\t(agent " << id << ") remaining_movement reset to " << max_turn_movement << std::endl;
    }
    void reset_remaining_battery () {
        remaining_battery = max_battery_life;
        std::cout << "\t\t(agent " << id << ") remaining_battery reset to " << max_battery_life << std::endl;
    }
    void reset_remaining_charge_time () {
        remaining_charge_time = recharge_time;
        std::cout << "\t\t(agent " << id << ") remaining_charge_time reset to " << recharge_time << std::endl;
    }
    void reset_remaining_coverage () {
        remaining_coverage = max_turn_coverage;
        std::cout << "\t\t(agent " << id << ") remaining_coverage reset to " << max_turn_coverage << std::endl;
    }
    void reset_score () {
        score = 0;
        std::cout << "\t\t(agent " << id << ") score reset to " << score << std::endl;
    }
    // . Update
    void update_location (int index) {
        location = index;
        std::cout << "\t\t(agent " << id << ") location updated to " << index << std::endl;
    }
    void update_score (int update) {
        score += update;
        std::cout << "\t\t(agent " << id << ") score updated by a value of " << update << " to " << score << std::endl;
    }

    int remaining_movement;
    int remaining_coverage;
    int remaining_battery;
    int remaining_charge_time;

private:
    std::string id;
    int type;
    bool can_reach_upper;
    int max_turn_movement;
    int max_turn_coverage;
    int max_battery_life;
    int recharge_time;
    int location;
    int score;
};

// . Drone, Quadruped, Gantry classes
class Drone : public Robot
{
public:
    Drone(std::string handle) : Robot(handle, 0, true, 5, 1, 2, 2) {}
};

class Quadruped : public Robot
{
public:
    Quadruped(std::string handle) : Robot(handle, 1, false, 3, 3, 10, 2) {}
};

class Gantry : public Robot
{
public:
    Gantry(std::string handle) : Robot(handle, 2, true, 1, 9, 30, 2) {}
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
