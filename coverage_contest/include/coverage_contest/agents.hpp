#ifndef GAME_PLAYERS_H
#define GAME_PLAYERS_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include <algorithm>
#include <random>
#include <any>

namespace agents
{
/** @class Robot
 * @brief contains and tracks static and dynamic robot state information and attributes
 */
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
        // std::cout << "\t\t(agent " << id << ") remaining_movement reset to " << max_turn_movement << std::endl;
    }
    void reset_remaining_battery () {
        remaining_battery = max_battery_life;
        // std::cout << "\t\t(agent " << id << ") remaining_battery reset to " << max_battery_life << std::endl;
    }
    void reset_remaining_charge_time () {
        remaining_charge_time = recharge_time;
        // std::cout << "\t\t(agent " << id << ") remaining_charge_time reset to " << recharge_time << std::endl;
    }
    void reset_remaining_coverage () {
        remaining_coverage = max_turn_coverage;
        // std::cout << "\t\t(agent " << id << ") remaining_coverage reset to " << max_turn_coverage << std::endl;
    }
    void reset_score () {
        score = 0;
        // std::cout << "\t\t(agent " << id << ") score reset to " << score << std::endl;
    }
    // . Update
    void update_location (int index) {
        location = index;
        // std::cout << "\t\t(agent " << id << ") location updated to " << index << std::endl;
    }
    void update_score (int update) {
        score += update;
        // std::cout << "\t\t(agent " << id << ") score updated by a value of " << update << " to " << score << std::endl;
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
/** @class Drone
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Drone : public Robot
{
public:
    // Drone(std::string handle) : Robot(handle, 0, true, 10, 2, 3, 2) {}
    Drone(std::string handle) : Robot(handle, 0, true, 10, 2, 3, 1) {}
};

/** @class Quadruped
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Quadruped : public Robot
{
public:
    // Quadruped(std::string handle) : Robot(handle, 1, false, 3, 2, 8, 2) {}
    Quadruped(std::string handle) : Robot(handle, 1, false, 3, 5, 8, 2) {}
};

/** @class Gantry
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Gantry : public Robot
{
public:
    // Gantry(std::string handle) : Robot(handle, 2, true, 1, 5, 30, 2) {}
    Gantry(std::string handle) : Robot(handle, 2, true, 10, 30, 1, 9) {}
};

// . Instantiation function
/** @struct Party
 * @brief contains a map of Robot players with keys that match the elements of a vector that tracks playing order 
 */
struct Party
{
    std::map<std::string, Robot> players;
    std::vector<std::string> playing_order;
};

/** Randmly shuffles a party
 * @brief randomly shuffles the playing order of a given party
 * @param party Party type object with a playing order to be shuffled 
 */
void randomShufflePlayingOrder (agents::Party &party)
{
    std::shuffle(std::begin(party.playing_order), std::end(party.playing_order), std::random_device());
    std::cout << "[Agents] Player turn order randomly shuffled as: \n";
    for (std::size_t i = 0; i < party.playing_order.size(); i++) {
        std::cout << "\t" << static_cast<int>(i) << ": " << party.playing_order[i] << std::endl;;
    }
}

/** Instantiates the party for the current game
 * @brief creates a party object with players and a playing order for a game
 * @param num_drones the number of Drone type players to instantiate
 * @param num_quadrupeds the number of Drone type players to instantiate
 * @param num_gantries the number of Drone type players to instantiate
 * @param random_shuffle tells whether the playing order should be randomly shuffled after instantiation or not
 * @return Party type, as map with players and an order in which they play
 */
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
