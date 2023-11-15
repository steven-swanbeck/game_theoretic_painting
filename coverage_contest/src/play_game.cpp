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

    play_random_game_server_ = nh_.advertiseService("play_random_game", &GamePlayer::playRandomGame, this);
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

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "play_game");
    GamePlayer game_player;
    return 0;
}
