#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    play_random_game_server_ = nh_.advertiseService("play_random_game", &GamePlayer::playRandomGame, this);
    test_mcts_server_ = nh_.advertiseService("test_mcts", &GamePlayer::testMCTS, this);
    test_marker_server_ = nh_.advertiseService("test_marker", &GamePlayer::testMarker, this);
    gantry_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/gantry_marker", 1, this);
    quadruped_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/quadruped_marker", 1, this);
    drone_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/drone_marker", 1, this);
    
    visualizer_ = new GameVisualizer(nh_);
    
    ROS_INFO_STREAM("[GamePlayer] Up and ready.");
    ros::spin();
}

void GamePlayer::visualizeTurn (TurnSequence sequence)
{
    while (!sequence.empty()) {
        Action action {sequence.front()};
        if (action.move_id != -1) {
            // - handle movements
            // movePlayerMarker();

        } else {
            // - handle repairs and add to score

        }
        sequence.pop();
    }
}

void GamePlayer::takeTurnMCTS ()
{
    if (manager_.hasSufficientBattery()) {
        int search_duration_ms, num_candidates, search_depth {};
        float uct_c {};
        if (!nh_.param<int>("/game_theoretic_painting/mcts/search_duration_ms", search_duration_ms, 1000));
        if (!nh_.param<int>("/game_theoretic_painting/mcts/num_candidates", num_candidates, 20));
        if (!nh_.param<int>("/game_theoretic_painting/mcts/search_depth", search_depth, 5));
        if (!nh_.param<float>("/game_theoretic_painting/mcts/uct_c", uct_c, 1.4142136));

        MCTS mcts {MCTS(manager_.board_, manager_.party_, manager_.player_turn_, search_duration_ms, num_candidates, search_depth, uct_c)};

        TurnSequence sequence {mcts.search()};
        manager_.printSequence(sequence);
        visualizeTurn(sequence);
        manager_.playSequence(sequence);
    }
    manager_.startNext();
}

bool GamePlayer::testMarker (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = "package://coverage_contest/models/meshes/drone.dae";
    marker.pose.position.x = 0;
    marker.pose.position.y = -3.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.b = 0.10f;
    marker.color.g = 0.10f;
    marker.color.r = 0.90;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    drone_visualizer_.publish(marker);

    marker.id = 1;
    marker.mesh_resource = "package://coverage_contest/models/meshes/quadruped.dae";
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.b = 0.15f;
    marker.color.g = 0.84f;
    marker.color.r = 0.84f;
    marker.color.a = 1.0;
    quadruped_visualizer_.publish(marker);

    marker.id = 2;
    marker.mesh_resource = "package://coverage_contest/models/meshes/gantry.dae";
    marker.pose.position.x = 0;
    marker.pose.position.y = 3.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.b = 0.84f;
    marker.color.g = 0.84f;
    marker.color.r = 0.30;
    marker.color.a = 1.0;
    gantry_visualizer_.publish(marker);
    return true;
}

bool GamePlayer::playRandomGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    loadGame();

    manager_.playRandomGame();

    res.success = true;
    res.message = "Randomly played a game to completion!";
    return res.success;
}

bool GamePlayer::testMCTS (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    loadGame();

    while (!manager_.isOver()) {
        takeTurnMCTS();
    }
    std::vector<std::string> winners {manager_.determineWinners()};
    std::cout << "[Manager]\n------------------------------------------------------------\nGame has reached terminal state after " << manager_.total_turns_  << " turns!" << "\n------------------------------------------------------------" << std::endl;
    std::cout << "\tWinner(s):";
    for (std::string winner : winners) {
        std::cout << winner << " (" << manager_.party_.players.at(winner).get_score() << "),";
    }
    std::cout << std::endl;

    res.success = true;
    res.message = "Played a series of moves using MCTS!";
    return res.success;
}

void GamePlayer::loadGame ()
{
    std::string ws_dir {};
    nh_.getParam("/game_theoretic_painting/paths/pkg_path", ws_dir);
    float movement_discretization, repair_discretization {};
    nh_.getParam("/game_theoretic_painting/board/movement/discretization", movement_discretization);
    nh_.getParam("/game_theoretic_painting/board/repair/discretization", repair_discretization);
    int n_drones, n_quadrupeds, n_gantries {};
    nh_.getParam("/game_theoretic_painting/party/n_drones", n_drones);
    nh_.getParam("/game_theoretic_painting/party/n_quadrupeds", n_quadrupeds);
    nh_.getParam("/game_theoretic_painting/party/n_gantries", n_gantries);
    int starting_location {};
    nh_.getParam("/game_theoretic_painting/party/starting_location", starting_location);
    
    manager_.instantiateBoard(ws_dir + "coverage_contest/models/clouds/revised/map.pcd", movement_discretization, ws_dir + "coverage_contest/models/clouds/revised/marked.pcd", repair_discretization);
    manager_.instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);
}

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
