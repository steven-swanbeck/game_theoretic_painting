#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    // std::string ws_dir {};
    // nh_.getParam("/game_theoretic_painting/paths/src_path", ws_dir);
    // float movement_discretization, repair_discretization {};
    // nh_.getParam("/game_theoretic_painting/board/movement/discretization", movement_discretization);
    // nh_.getParam("/game_theoretic_painting/board/repair/discretization", repair_discretization);
    // int n_drones, n_quadrupeds, n_gantries {};
    // nh_.getParam("/game_theoretic_painting/party/n_drones", n_drones);
    // nh_.getParam("/game_theoretic_painting/party/n_quadrupeds", n_quadrupeds);
    // nh_.getParam("/game_theoretic_painting/party/n_gantries", n_gantries);
    // int starting_location {};
    // nh_.getParam("/game_theoretic_painting/party/starting_location", starting_location);
    


    // manager_.instantiateBoard(ws_dir + "/models/clouds/revised/map.pcd", movement_discretization, ws_dir + "/models/clouds/revised/marked.pcd", repair_discretization);
    // manager_.instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

    // // GameManager alt_manager {manager_.board_, manager_.party_};
    
    // manager_.playRandomGame();

    // // std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
    // // alt_manager.playRandomGame();

    
    // // manager_.listMovesfromNode();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.takeRandomTurn();
    // // manager_.printMovesfromState(0);
    // // manager_.printMovesfromState(2);
    // // manager_.printMovesfromState(4);
    // // manager_.testRandomTurns(10);
    // // manager_.startNext();
    // // manager_.takeTurn();
    // // manager_.startNext();
    // // manager_.takeTurn();

    // simulateGame();

    // std::cout << "[Play game] Passed game tests." << std::endl;

    // Create visualizer object
    visualizer_ = new GameVisualizer(nh_);

    play_random_game_server_ = nh_.advertiseService("play_random_game", &GamePlayer::playRandomGame, this);

    // Create services for the Game Visualizer
    test_game_visualizer_ = nh_.advertiseService("test_game_visualizer", &GamePlayer::testGameVisualizer, this);

    clear_game_visualizer_ = nh_.advertiseService("clear_game_visualizer", &GamePlayer::clearGameVisualizer, this);

    ROS_INFO_STREAM("[GamePlayer] Up and ready.");

    ros::spin();
}

bool GamePlayer::playRandomGame (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::string ws_dir {};
    nh_.getParam("/game_theoretic_painting/paths/src_path", ws_dir);
    float movement_discretization, repair_discretization {};
    nh_.getParam("/game_theoretic_painting/board/movement/discretization", movement_discretization);
    nh_.getParam("/game_theoretic_painting/board/repair/discretization", repair_discretization);
    int n_drones, n_quadrupeds, n_gantries {};
    nh_.getParam("/game_theoretic_painting/party/n_drones", n_drones);
    nh_.getParam("/game_theoretic_painting/party/n_quadrupeds", n_quadrupeds);
    nh_.getParam("/game_theoretic_painting/party/n_gantries", n_gantries);
    int starting_location {};
    nh_.getParam("/game_theoretic_painting/party/starting_location", starting_location);
    
    manager_.instantiateBoard(ws_dir + "/models/clouds/revised/map.pcd", movement_discretization, ws_dir + "/models/clouds/revised/marked.pcd", repair_discretization);
    manager_.instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

    manager_.playRandomGame();

    res.success = true;
    res.message = "Randomly played a game to completion!";
    return res.success;
}

void GamePlayer::simulateGame ()
{
    GameManager alt_manager {manager_.board_, manager_.party_, 0};
    alt_manager.playRandomGame();

    std::vector<std::string> winners {alt_manager.determineWinners()};

    for (auto winner : winners) {
        std::cout << winner << ": " << alt_manager.party_.players.at(winner).get_score() << std::endl;
    }

    // std::cout << alt_manager.party_.players.at("gantry_0").get_score() << std::endl;
}

// TODO Add these to the YAML file
#define MAP_MESH    "package://coverage_contest/models/meshes/URSA.stl"
#define MAP_POINTS  "/home/daniel/Research/Workspaces/ASE_389/src/game_theoretic_painting/coverage_contest/models/clouds/revised/map.pcd"

#define REPAIR_POINTS       "/home/daniel/Research/Workspaces/ASE_389/src/game_theoretic_painting/coverage_contest/models/clouds/revised/marked.pcd"

#define QUADRUPED_MESH      "package://coverage_contest/models/meshes/spot_body.dae"


bool GamePlayer::testGameVisualizer (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ros::Rate r(1);

    // Map environment
    visualizer_->addEnvironment("map", std::vector<int16_t>{75, 75, 75});
    
    visualizer_->addEnvironmentMarker("map", MAP_MESH, std::vector<float_t>{40.0, 0.0, -15.0});
    visualizer_->publishEnvironmentMarker("map");

    visualizer_->addEnvironmentPoints("map", MAP_POINTS);
    visualizer_->publishEnvironmentPoints("map");

    // Repair environment
    visualizer_->addEnvironment("repair", std::vector<int16_t>{255, 255, 255});

    visualizer_->addEnvironmentPoints("repair", REPAIR_POINTS);
    visualizer_->publishEnvironmentPoints("repair");

    // Creating point cloud msg from PCD file for testing.
    sensor_msgs::PointCloud2 repair_points;
    pcl::PCDReader reader;
    PointCloud::Ptr pcl_points (new PointCloud);
    reader.read(REPAIR_POINTS, *pcl_points);
    pcl::toROSMsg(*pcl_points, repair_points);

    // Quadruped Players
    visualizer_->addPlayer("quadruped0", std::vector<int16_t>{255, 0, 0}, QUADRUPED_MESH);
    visualizer_->addPlayer("quadruped1", std::vector<int16_t>{0, 255, 0}, QUADRUPED_MESH);

    visualizer_->addPlayerPoints("quadruped0", repair_points);
    visualizer_->publishPlayerPoints("quadruped0");

    for (uint8_t i = 0; i < 5; i++) {
        visualizer_->movePlayerMarker("quadruped0", std::vector<float_t>{(float)0.5 * i, 0.0, 0.5});
        visualizer_->publishPlayerMarker("quadruped0");

        visualizer_->movePlayerMarker("quadruped1", std::vector<float_t>{0.0, (float)0.5 * i, 0.5});
        visualizer_->publishPlayerMarker("quadruped1");

        r.sleep();
    }

    res.success = true;
    res.message = "Test Completed!";
    return res.success;
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

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
