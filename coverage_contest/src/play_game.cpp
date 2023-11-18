#include "coverage_contest/play_game.h"

#define environment_mesh_ "package://coverage_contest/models/meshes/facility.stl"
#define drone_mesh_ "package://coverage_contest/models/meshes/drone.dae"
#define quadruped_mesh_ "package://coverage_contest/models/meshes/quadruped.dae"
#define gantry_mesh_ "/coverage_contest/models/meshes/gantry.dae"

GamePlayer::GamePlayer ()
{
    play_random_game_server_ = nh_.advertiseService("play_random_game", &GamePlayer::playRandomGame, this);
    test_mcts_server_ = nh_.advertiseService("test_mcts", &GamePlayer::testMCTS, this);
    test_marker_server_ = nh_.advertiseService("test_marker", &GamePlayer::testMarker, this);
    gantry_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/gantry_marker", 1, this);
    quadruped_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/quadruped_marker", 1, this);
    drone_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/drone_marker", 1, this);

    map_visualizer_ = nh_.advertise<sensor_msgs::PointCloud2>("/map", 1, this);
    marked_visualizer_ = nh_.advertise<sensor_msgs::PointCloud2>("/marked", 1, this);
    environment_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/environment_marker", 1, this);

    clear_game_visualizer_ = nh_.advertiseService("test_vis_clear", &GamePlayer::clearGameVisualizer, this);
    
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
            // . move appropriate player marker to target location
            visualizer_->movePlayerMarker(manager_.playingNow(), getLocationVector(action.move_id));
        } else {
            // - handle repairs and add to score
            visualizer_->addPlayerPoints(manager_.playingNow(), manager_.board_.repair_spaces.at(action.repair_id).cloud);
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
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/pkg_path", ws_dir, "/home/steven/game_theoretic_painting/src/"));
    float movement_discretization, repair_discretization {};
    if (!nh_.param<float>("/game_theoretic_painting/board/movement/discretization", movement_discretization, 3.0));
    if (!nh_.param<float>("/game_theoretic_painting/board/repair/discretization", repair_discretization, 1.0));
    int n_drones, n_quadrupeds, n_gantries {};
    if (!nh_.param<int>("/game_theoretic_painting/party/n_drones", n_drones, 1));
    if (!nh_.param<int>("/game_theoretic_painting/party/n_quadrupeds", n_quadrupeds, 1));
    if (!nh_.param<int>("/game_theoretic_painting/party/n_gantries", n_gantries, 1));
    int starting_location {};
    if (!nh_.param<int>("/game_theoretic_painting/party/starting_location", starting_location, 0));

    // if (!nh_.param<std::string>("/game_theoretic_painting/paths/meshes/environment", environment_mesh_, "package://coverage_contest/models/meshes/facility.stl"));
    // if (!nh_.param<std::string>("/game_theoretic_painting/paths/meshes/players/drone", drone_mesh_, "package://coverage_contest/models/meshes/drone.dae"));
    // if (!nh_.param<std::string>("/game_theoretic_painting/paths/meshes/players/quadruped", quadruped_mesh_, "package://coverage_contest/models/meshes/quadruped.dae"));
    // if (!nh_.param<std::string>("/game_theoretic_painting/paths/meshes/players/gantry", gantry_mesh_, "package://coverage_contest/models/meshes/gantry.dae"));

    std::string rel_map_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/map", rel_map_dir, "/coverage_contest/models/clouds/revised/map.pcd"));
    std::string map_dir {ws_dir + rel_map_dir};

    std::string rel_marked_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/marked", rel_marked_dir, "/coverage_contest/models/clouds/revised/marked.pcd"));
    std::string marked_dir {ws_dir + rel_marked_dir};

    // manager_.instantiateBoard(ws_dir + "coverage_contest/models/clouds/revised/map.pcd", movement_discretization, ws_dir + "coverage_contest/models/clouds/revised/marked.pcd", repair_discretization);
    
    manager_.instantiateBoard(map_dir, movement_discretization, map_, marked_dir, repair_discretization, marked_);
    manager_.instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

    map_.header.frame_id = "map";
    marked_.header.frame_id = "map";
    instantiateVisualizer ();
}

void GamePlayer::instantiateVisualizer ()
{
    bool user_input_color {};
    if (!nh_.param<bool>("/game_theoretic_painting/party/user_input_color", user_input_color, false));

    map_visualizer_.publish(map_);
    marked_visualizer_.publish(marked_);

    // - map environment
    visualizer_->addEnvironment("environment", std::vector<int16_t>{100, 100, 100});
    visualizer_->addEnvironmentMarker("environment", environment_mesh_, std::vector<float_t>{40.0, 0.0, -15.0});
    visualizer_->publishEnvironmentMarker("environment");
    visualizer_->addEnvironmentPoints("environment", map_);
    visualizer_->publishEnvironmentPoints("environment");

    // - repair environment
    visualizer_->addEnvironment("repair", std::vector<int16_t>{255, 255, 255});
    visualizer_->addEnvironmentPoints("repair", marked_);
    visualizer_->publishEnvironmentPoints("repair");

    // - create player visualizations
    for (std::size_t i = 0; i < manager_.party_.playing_order.size(); i++) {
        switch (manager_.party_.players.at(manager_.party_.playing_order[i]).get_type()) {
            case 0:
            {
                std::cout << "Trying to add marker for drone type" << std::endl;
                // visualizer_->addPlayer(manager_.party_.playing_order[i], std::vector<int16_t>{255, 0, 0}, "package://coverage_contest/models/meshes/drone.dae");
                visualizer_->addPlayer(manager_.party_.playing_order[i], "package://coverage_contest/models/meshes/drone.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_.party_.playing_order[i], getLocationVector(manager_.party_.playing_order[i]));
                break;
            }
            case 1:
            {
                std::cout << "Trying to add marker for quadruped type" << std::endl;
                visualizer_->addPlayer(manager_.party_.playing_order[i], "package://coverage_contest/models/meshes/quadruped.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_.party_.playing_order[i], 
                getLocationVector(manager_.party_.playing_order[i]));
                break;
            }
            case 2:
            {
                std::cout << "Trying to add marker for gantry type" << std::endl;
                visualizer_->addPlayer(manager_.party_.playing_order[i], "package://coverage_contest/models/meshes/gantry.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_.party_.playing_order[i], getLocationVector(manager_.party_.playing_order[i]));
                break;
            }
        }
    }
}

std::vector<float_t> GamePlayer::getLocationVector (const int &id)
{
    return pointToVector(manager_.board_.movement_spaces.at(id).centroid);
}

std::vector<float_t> GamePlayer::getLocationVector (const std::string &id)
{
    int location {manager_.party_.players.at(id).get_location()};
    return pointToVector(manager_.board_.movement_spaces.at(location).centroid);
}

std::vector<float_t> GamePlayer::pointToVector (const PointT &point)
{
    std::vector<float_t> location {point.x, point.y, point.z + static_cast<float>(0.8)};
    return location;
}

// bool GamePlayer::testGameVisualizer (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
// {
//     ros::Rate r(1);

//     // Map environment
//     visualizer_->addEnvironment("map", std::vector<int16_t>{75, 75, 75});
    
//     visualizer_->addEnvironmentMarker("map", environment_mesh_, std::vector<float_t>{40.0, 0.0, -15.0});
//     visualizer_->publishEnvironmentMarker("map");

//     visualizer_->addEnvironmentPoints("map", map_);
//     visualizer_->publishEnvironmentPoints("map");

//     // Repair environment
//     visualizer_->addEnvironment("repair", std::vector<int16_t>{255, 255, 255});

//     visualizer_->addEnvironmentPoints("repair", marked_);
//     visualizer_->publishEnvironmentPoints("repair");



//     // Quadruped Players
//     visualizer_->addPlayer("quadruped0", std::vector<int16_t>{255, 0, 0}, "package://coverage_contest/models/meshes/quadruped.dae");
//     visualizer_->addPlayer("quadruped1", std::vector<int16_t>{255, 0, 0}, "package://coverage_contest/models/meshes/quadruped.dae");
//     // std::string test {"package://coverage_contest/models/meshes/quadruped.dae"};
//     // visualizer_->addPlayer("quadruped0", std::vector<int16_t>{0, 255, 0}, test);
//     // visualizer_->addPlayer("quadruped0", std::vector<int16_t>{0, 255, 0}, quadruped_mesh_);
//     // visualizer_->addPlayer("quadruped1", std::vector<int16_t>{0, 255, 0}, quadruped_mesh_);

//     // visualizer_->addPlayerPoints("quadruped0", marked_);
//     // visualizer_->publishPlayerPoints("quadruped0");

//     for (uint8_t i = 0; i < 5; i++) {
//         visualizer_->movePlayerMarker("quadruped0", std::vector<float_t>{(float)0.5 * i, 0.0, 0.5});
//         visualizer_->publishPlayerMarker("quadruped0");

//         visualizer_->movePlayerMarker("quadruped1", std::vector<float_t>{0.0, (float)0.5 * i, 0.5});
//         visualizer_->publishPlayerMarker("quadruped1");

//         r.sleep();
//     }

//     res.success = true;
//     res.message = "Test Completed!";
//     return res.success;
// }

bool GamePlayer::clearGameVisualizer (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ros::Rate r(0.2);

    // Clear visualizer
    visualizer_->clearVisualizer();

    r.sleep();

    // Clear objects
    visualizer_->clearObjects();

    res.success = true;
    res.message = "Game Visualizer Cleared!";
    return res.success;
}

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
