#include "coverage_contest/play_game.h"

#define environment_mesh_ "package://coverage_contest/models/meshes/facility.stl"
#define drone_mesh_ "package://coverage_contest/models/meshes/drone.dae"
#define quadruped_mesh_ "package://coverage_contest/models/meshes/quadruped.dae"
#define gantry_mesh_ "package://coverage_contest/models/meshes/gantry.dae"

GamePlayer::GamePlayer ()
{
    play_random_game_server_ = nh_.advertiseService("play_random_game", &GamePlayer::playRandomGame, this);
    play_game_server_ = nh_.advertiseService("play_game", &GamePlayer::playGame, this);
    play_n_games_server_ = nh_.advertiseService("play_n_games", &GamePlayer::playNGames, this);
    exhaustive_search_server_ = nh_.advertiseService("exhaustive_search", &GamePlayer::exhaustiveSearch, this);
    custom_search_server_ = nh_.advertiseService("custom_search", &GamePlayer::customSearch, this);

    test_marker_server_ = nh_.advertiseService("test_marker", &GamePlayer::testMarker, this);
    gantry_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/gantry_marker", 1, this);
    quadruped_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/quadruped_marker", 1, this);
    drone_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/drone_marker", 1, this);

    map_visualizer_ = nh_.advertise<sensor_msgs::PointCloud2>("/map", 1, this);
    marked_visualizer_ = nh_.advertise<sensor_msgs::PointCloud2>("/marked", 1, this);
    environment_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/environment_marker", 1, this);

    reset_game_ = nh_.advertiseService("reset_game", &GamePlayer::resetGame, this);
    clear_game_visualizer_ = nh_.advertiseService("test_vis_clear", &GamePlayer::clearGameVisualizer, this);
    
    visualizer_ = new GameVisualizer(nh_);
    
    ROS_INFO_STREAM("[GamePlayer] Up and ready.");
    ros::spin();
}

void GamePlayer::visualizeTurn (TurnSequence sequence)
{
    ros::Rate r(1);
    int turn_start_location {manager_->party_.players.at(manager_->playingNow()).get_location()};
    while (!sequence.empty()) {
        Action action {sequence.front()};
        if (action.move_id != -1) {
            // - handle movements, move appropriate player marker to target location
            interpolatePath(turn_start_location, action.move_id);
            turn_start_location = action.move_id;
        } else {
            // - handle repairs and add to score
            visualizer_->addPlayerPoints(manager_->playingNow(), manager_->board_.repair_spaces.at(action.repair_id).cloud);
            r.sleep();
        }
        sequence.pop();
    }
}

void GamePlayer::interpolatePath (const int &start_node, const int &end_node)
{
    ros::Rate r(20);
    int num_steps {10};
    switch (manager_->party_.players.at(manager_->playingNow()).get_type()) {
        case 0:
        {
            num_steps = 10;
            break;
        }
        case 1:
        {
            num_steps = 15;
            break;
        }
        case 2:
        {
            num_steps = 20;
            break;
        }
    }

    std::vector<float_t> location {getLocationVector(start_node)};
    std::vector<float_t> goal {getLocationVector(end_node)};

    float_t x_step = (goal[0] - location[0]) / num_steps;
    float_t y_step = (goal[1] - location[1]) / num_steps;
    float_t z_step = (goal[2] - location[2]) / num_steps;

    for (std::size_t i = 0; i < num_steps; i++) {
        location[0] += x_step;
        location[1] += y_step;
        location[2] += z_step;
        visualizer_->movePlayerMarker(manager_->playingNow(), location);
        r.sleep();
    }
}

void GamePlayer::takeTurnMCTS (bool should_visualize)
{
    if (manager_->hasSufficientBattery()) {
        int search_duration_ms, num_candidates, search_depth {};
        float uct_c {};
        if (!nh_.param<int>("/game_theoretic_painting/mcts/search_duration_ms", search_duration_ms, 1000));
        if (!nh_.param<int>("/game_theoretic_painting/mcts/num_candidates", num_candidates, 20));
        if (!nh_.param<int>("/game_theoretic_painting/mcts/search_depth", search_depth, 5));
        if (!nh_.param<float>("/game_theoretic_painting/mcts/uct_c", uct_c, 1.4142136));

        MCTS mcts {MCTS(manager_->board_, manager_->party_, manager_->player_turn_, search_duration_ms, num_candidates, search_depth, uct_c)};

        TurnSequence sequence {mcts.search()};
        // manager_.printSequence(sequence);
        if (should_visualize) {
            visualizeTurn(sequence);
        }
        manager_->playSequence(sequence);
        manager_->printScoreboard();
    }
    manager_->startNext();
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

    manager_->playRandomGame();

    res.success = true;
    res.message = "Randomly played a game to completion!";
    return res.success;
}

bool GamePlayer::playGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    loadGame();
    playMCTSGame();

    res.success = true;
    res.message = "Played a series of moves using MCTS!";
    return res.success;
}

void GamePlayer::playMCTSGame (bool should_visualize)
{
    while (!manager_->isOver()) {
        takeTurnMCTS(should_visualize);
    }
    std::vector<std::string> winners {manager_->determineWinners()};
    std::cout << "[Manager]\n------------------------------------------------------------\nGame has reached terminal state after " << manager_->total_turns_  << " turns!" << "\n------------------------------------------------------------" << std::endl;
    std::cout << "\tWinner(s):";
    for (std::string winner : winners) {
        std::cout << winner << " (" << manager_->party_.players.at(winner).get_score() << "),";
    }
    std::cout << std::endl;
}

bool GamePlayer::customSearch (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::vector<int> drone_schedule {1, 2, 3, 4, 5,
                                    0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    1, 2, 3, 4, 5};
    std::vector<int> quadruped_schedule {0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    1, 2, 3, 4, 5, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5};
    std::vector<int> gantry_schedule {0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    0, 0, 0, 0, 0, 
                                    1, 2, 3, 4, 5, 
                                    1, 2, 3, 4, 5, 
                                    1, 2, 3, 4, 5};

    assert((drone_schedule.size() == quadruped_schedule.size()) && (quadruped_schedule.size() == gantry_schedule.size()));

    int num_games {};
    if (!nh_.param<int>("/game_theoretic_painting/party/num_games", num_games, 2));

    std::ofstream results_log;
    std::string ws_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/pkg_path", ws_dir, "/home/steven/game_theoretic_painting/src/"));
    std::string file_name {ws_dir + "coverage_contest/logs/" + std::to_string(ros::Time::now().toSec()) + ".csv"};
    results_log.open(file_name);
    std::cout << "Recording game results to " << file_name << std::endl;

    results_log.close();

    for (std::size_t i = 0; i < drone_schedule.size(); i++) {
        for (std::size_t j = 0; j < num_games; j++) {
            loadGame(drone_schedule[i], quadruped_schedule[i], gantry_schedule[i]);
            playMCTSGame(false);

            results_log.open(file_name, std::ios::out | std::ios::app);
            results_log << ", " << "Id" << ", " << "Score" << ", " << "\n";
            for (std::size_t l = 0; l < manager_->party_.playing_order.size(); l++) {
                results_log << ", " << manager_->party_.players.at(manager_->party_.playing_order[l]).get_id() << ", " << manager_->party_.players.at(manager_->party_.playing_order[l]).get_score() << ", " << "\n";
            }
            results_log << ", " << "Total Turns " << ", " << manager_->total_turns_ << ", ";
            results_log << "\n" << "\n";
            results_log.close();

            std_srvs::Trigger srv;
            resetGame(srv.request, srv.response);

            std::cout << "Finished game " << static_cast<int>(j) << " for schedule index " << static_cast<int>(i) << std::endl;
        }
    }
    res.success = true;
    return res.success;
}

bool GamePlayer::exhaustiveSearch (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::ofstream results_log;
    std::string ws_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/pkg_path", ws_dir, "/home/steven/game_theoretic_painting/src/"));
    std::string file_name {ws_dir + "coverage_contest/logs/" + std::to_string(ros::Time::now().toSec()) + ".csv"};
    results_log.open(file_name);
    std::cout << "Recording game results to " << file_name << std::endl;

    int num_games {};
    if (!nh_.param<int>("/game_theoretic_painting/party/num_games", num_games, 2));

    results_log << "Search Ceiling" << ", " << num_games << "\n\n";
    results_log.close();

    for (std::size_t i = 0; i < num_games; i++) {
        for (std::size_t j = 0; j < num_games; j++) {
            for (std::size_t k = 0; k < num_games; k++) {

                results_log.open(file_name, std::ios::out | std::ios::app);
                
                loadGame (static_cast<int>(i + 1), static_cast<int>(j + 1), static_cast<int>(k + 1));
                playMCTSGame(false);

                results_log << "n_drones: " << ", " << static_cast<int>(i + 1) << ", " << "n_quadrupeds: " << ", " << static_cast<int>(j + 1) << ", " << "n_gantries: " << ", " << static_cast<int>(k + 1) << ", " << "\n";
                results_log << ", " << "Id" << ", " << "Score" << ", " << "\n";
                
                for (std::size_t l = 0; l < manager_->party_.playing_order.size(); l++) {
                    results_log << ", " << manager_->party_.players.at(manager_->party_.playing_order[l]).get_id() << ", " << manager_->party_.players.at(manager_->party_.playing_order[l]).get_score() << ", " << "\n";
                }
                results_log << ", " << "Total Turns " << ", " << manager_->total_turns_ << ", ";
                results_log << "\n" << "\n";
                results_log.close();

                std_srvs::Trigger srv;
                resetGame(srv.request, srv.response);
            }
        }
    }
    res.success = true;
    return res.success;
}

bool GamePlayer::playNGames (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // - create an output file (csv or txt?)
    std::ofstream results_log;
    std::string ws_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/pkg_path", ws_dir, "/home/steven/game_theoretic_painting/src/"));
    std::string file_name {ws_dir + "coverage_contest/logs/" + std::to_string(ros::Time::now().toSec()) + ".csv"};
    results_log.open(file_name);
    std::cout << "Recording game results to " << file_name << std::endl;

    // - get param for number of games to play
    int num_games {};
    if (!nh_.param<int>("/game_theoretic_painting/party/num_games", num_games, 2));

    results_log << "Num Games" << ", " << num_games << "\n\n";
    results_log.close();

    // - iterate that many times, but with the added clearing after each has concluded
    for (std::size_t i = 0; i < num_games; i++) {
        results_log.open(file_name, std::ios::out | std::ios::app);

        loadGame();

        std_srvs::Trigger srv;
        playGame(srv.request, srv.response);

        results_log << static_cast<int>(i) << ", " << "Id" << ", " << "Score" << ", " << "\n";
        for (std::size_t j = 0; j < manager_->party_.playing_order.size(); j++) {
            results_log << ", " << manager_->party_.players.at(manager_->party_.playing_order[j]).get_id() << ", " << manager_->party_.players.at(manager_->party_.playing_order[j]).get_score() << ", " << "\n";
        }
        results_log << ", " << "Total Turns " << ", " << manager_->total_turns_ << ", ";
        results_log << "\n" << "\n";
        results_log.close();

        resetGame(srv.request, srv.response);
        std::cout << "Finished simulating game " << static_cast<int>(i) << "!\n\n\n" << std::endl;
    }
    res.success = true;
    return res.success;
}

bool GamePlayer::resetGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    delete manager_;
    std_srvs::Trigger srv;
    clearGameVisualizer(srv.request, srv.response);
    res.success = true;
    return res.success;
}

void GamePlayer::loadGame (int n_drones, int n_quadrupeds, int n_gantries)
{
    manager_ = new GameManager();

    std::string ws_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/pkg_path", ws_dir, "/home/steven/game_theoretic_painting/src/"));
    float movement_discretization, repair_discretization {};
    if (!nh_.param<float>("/game_theoretic_painting/board/movement/discretization", movement_discretization, 3.0));
    if (!nh_.param<float>("/game_theoretic_painting/board/repair/discretization", repair_discretization, 1.0));
    int starting_location {};
    if (!nh_.param<int>("/game_theoretic_painting/party/starting_location", starting_location, 0));
    std::string rel_map_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/map", rel_map_dir, "/coverage_contest/models/clouds/revised/map.pcd"));
    std::string map_dir {ws_dir + rel_map_dir};
    std::string rel_marked_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/marked", rel_marked_dir, "/coverage_contest/models/clouds/revised/marked.pcd"));
    std::string marked_dir {ws_dir + rel_marked_dir};

    manager_->instantiateBoard(map_dir, movement_discretization, map_, marked_dir, repair_discretization, marked_);
    manager_->instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

    map_.header.frame_id = "map";
    marked_.header.frame_id = "map";
    instantiateVisualizer ();
}

void GamePlayer::loadGame ()
{
    manager_ = new GameManager();

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

    std::string rel_map_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/map", rel_map_dir, "/coverage_contest/models/clouds/revised/map.pcd"));
    std::string map_dir {ws_dir + rel_map_dir};

    std::string rel_marked_dir {};
    if (!nh_.param<std::string>("/game_theoretic_painting/paths/clouds/raw/marked", rel_marked_dir, "/coverage_contest/models/clouds/revised/marked.pcd"));
    std::string marked_dir {ws_dir + rel_marked_dir};

    manager_->instantiateBoard(map_dir, movement_discretization, map_, marked_dir, repair_discretization, marked_);
    manager_->instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

    map_.header.frame_id = "map";
    marked_.header.frame_id = "map";
    instantiateVisualizer ();
}

void GamePlayer::instantiateVisualizer ()
{
    bool user_input_color {};
    if (!nh_.param<bool>("/game_theoretic_painting/party/user_input_color", user_input_color, false));
    bool use_color_for_player {};
    if (!nh_.param<bool>("/game_theoretic_painting/party/use_color_for_player", use_color_for_player, true));
    float environment_alpha {};
    if (!nh_.param<float>("/game_theoretic_painting/board/mesh_alpha", environment_alpha, 1.0));

    map_visualizer_.publish(map_);
    marked_visualizer_.publish(marked_);

    // - map environment
    visualizer_->addEnvironment("environment", std::vector<int16_t>{100, 100, 100});
    visualizer_->addEnvironmentMarker("environment", environment_mesh_, std::vector<float_t>{40.0, 0.0, -15.0}, environment_alpha);
    visualizer_->publishEnvironmentMarker("environment");
    visualizer_->addEnvironmentPoints("environment", map_);
    visualizer_->publishEnvironmentPoints("environment");

    // - repair environment
    visualizer_->addEnvironment("repair", std::vector<int16_t>{255, 255, 255});
    visualizer_->addEnvironmentPoints("repair", marked_);
    visualizer_->publishEnvironmentPoints("repair");

    // - create player visualizations
    for (std::size_t i = 0; i < manager_->party_.playing_order.size(); i++) {
        switch (manager_->party_.players.at(manager_->party_.playing_order[i]).get_type()) {
            case 0:
            {
                std::cout << "Trying to add marker for drone type" << std::endl;
                visualizer_->addPlayer(manager_->party_.playing_order[i], "package://coverage_contest/models/meshes/drone.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_->party_.playing_order[i], getLocationVector(manager_->party_.playing_order[i]));
                break;
            }
            case 1:
            {
                std::cout << "Trying to add marker for quadruped type" << std::endl;
                visualizer_->addPlayer(manager_->party_.playing_order[i], "package://coverage_contest/models/meshes/quadruped.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_->party_.playing_order[i], 
                getLocationVector(manager_->party_.playing_order[i]));
                break;
            }
            case 2:
            {
                std::cout << "Trying to add marker for gantry type" << std::endl;
                visualizer_->addPlayer(manager_->party_.playing_order[i], "package://coverage_contest/models/meshes/gantry.dae", user_input_color);
                visualizer_->movePlayerMarker(manager_->party_.playing_order[i], getLocationVector(manager_->party_.playing_order[i]));
                break;
            }
        }
    }
}

std::vector<float_t> GamePlayer::getLocationVector (const int &id)
{
    return pointToVector(manager_->board_.movement_spaces.at(id).centroid);
}

std::vector<float_t> GamePlayer::getLocationVector (const std::string &id)
{
    int location {manager_->party_.players.at(id).get_location()};
    return pointToVector(manager_->board_.movement_spaces.at(location).centroid);
}

std::vector<float_t> GamePlayer::pointToVector (const PointT &point)
{
    std::vector<float_t> location {point.x, point.y, point.z + static_cast<float>(0.8)};
    return location;
}

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

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
