#ifndef GAME_PLAYER_H
#define GAME_PLAYER_H


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

// #include "coverage_contest/common.h"
// #include "coverage_contest/point.h"
// #include "board.hpp"
// #include "agents.hpp"
// #include "game_manager.h"
// #include "mcts.h"
#include "game_visualizer.h"
// #include <fstream>


class GamePlayer : public rclcpp::Node {
  public:
    GamePlayer();

  private:
    void clearGameVisualizer(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_game_visualizer_service_;

};


#endif // GAME_PLAYER_H
