#include "coverage_contest/play_game.h"

#define environment_mesh_ "package://coverage_contest/models/meshes/facility.stl"
#define drone_mesh_ "package://coverage_contest/models/meshes/drone.dae"
#define quadruped_mesh_ "package://coverage_contest/models/meshes/quadruped.dae"
#define gantry_mesh_ "package://coverage_contest/models/meshes/gantry.dae"

GameVisualizer *visualizer_;

GamePlayer::GamePlayer() : Node("play_game") {
  // Create services for the TODO

  // Create services for the game visualizer
  clear_game_visualizer_service_ = this->create_service<std_srvs::srv::Trigger>("~/clear_game_visualizer", std::bind(&GamePlayer::clearGameVisualizer, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "[GamePlayer] Up and ready.");
}

void GamePlayer::clearGameVisualizer(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Clear visualizer
  // TODO Uncomment
  // visualizer_->clearVisualizer();

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  response->success = true;
  response->message = "Game Visualizer cleared";
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePlayer>();
  visualizer_ = new GameVisualizer(node);
  // rclcpp::spin(std::make_shared<GamePlayer>());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
